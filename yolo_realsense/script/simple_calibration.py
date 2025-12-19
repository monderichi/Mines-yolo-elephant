#!/usr/bin/env python3
"""
Robust Eye-to-Hand Calibration with Median Filtering
(Camera Fixed, Moving Marker on Robot)

Improvements:
- Uses Median Filtering to reject outliers (robust to jitter)
- Uses Sub-pixel corner refinement for better accuracy
- Single-threaded main loop to prevent ROS2 Python threading crashes
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import json
from datetime import datetime
import time
import sys
import select


class PoseStabilizer:
    """Buffers poses and computes robust median/average."""
    def __init__(self, window_size=30):
        self.window_size = window_size
        self.tvecs = []
        self.quats = []
        
    def add_sample(self, tvec, rvec):
        self.tvecs.append(tvec.flatten())
        # Convert rvec to quaternion
        rmat, _ = cv2.Rodrigues(rvec)
        q = R.from_matrix(rmat).as_quat()
        self.quats.append(q)
        
    def is_full(self):
        return len(self.tvecs) >= self.window_size
        
    def clear(self):
        self.tvecs = []
        self.quats = []
        
    def get_stable_pose(self):
        if not self.tvecs:
            return None, None
            
        # 1. Reject outliers using Median Absolute Deviation (MAD) or simple Median
        # For simplicity and robustness, we use the component-wise Median
        tvecs_np = np.array(self.tvecs)
        median_tvec = np.median(tvecs_np, axis=0)
        
        # 2. Filter out points far from median (simple outlier rejection)
        # Distance from median
        dists = np.linalg.norm(tvecs_np - median_tvec, axis=1)
        # Keep samples within 2 standard deviations or a fixed threshold (e.g. 5cm)
        # Here we just take the nearest 60% of samples to the median to be safe
        n_keep = max(1, int(len(self.tvecs) * 0.6))
        keep_indices = np.argsort(dists)[:n_keep]
        
        clean_tvecs = tvecs_np[keep_indices]
        final_tvec = np.mean(clean_tvecs, axis=0)
        
        # 3. Rotation averaging
        # Simple average of quaternions (renormalized) is usually sufficient for small variation
        quats_np = np.array(self.quats)
        clean_quats = quats_np[keep_indices]
        
        # Handle quaternion sign flips (q and -q are same rotation)
        ref_quat = clean_quats[0]
        for i in range(len(clean_quats)):
            if np.dot(clean_quats[i], ref_quat) < 0:
                clean_quats[i] = -clean_quats[i]
                
        final_quat = np.mean(clean_quats, axis=0)
        final_quat /= np.linalg.norm(final_quat)
        
        std_dev = np.std(clean_tvecs, axis=0)
        
        return final_tvec, final_quat, std_dev


class RobustCalibration(Node):
    def __init__(self):
        super().__init__('robust_calibration')
        
        # Parameters
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.030)
        self.declare_parameter('sample_frames', 45)  # Increased for stability
        
        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.sample_frames = self.get_parameter('sample_frames').get_parameter_value().integer_value
        
        # Detection
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        # Enable subpixel refinement
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.bridge = CvBridge()
        
        # Camera
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame_id = 'camera_color_optical_frame'
        
        # State
        self.collecting = False
        self.stabilizer = PoseStabilizer(self.sample_frames)
        self.latest_interaction_tf = None
        
        # Samples (T_base_cam)
        self.calibration_samples = []
        
        # ROS
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Robust Eye-to-Hand Calibration (Median Filtered)')
        self.get_logger().info(f'Marker: {self.marker_id}, Size: {self.marker_size*1000}mm')
        self.get_logger().info(f'Sampling: {self.sample_frames} frames per pose')
        self.get_logger().info('='*60)

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_frame_id = msg.header.frame_id

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
            
            if ids is not None:
                for i, mid in enumerate(ids.flatten()):
                    if mid == self.marker_id:
                        half = self.marker_size / 2.0
                        obj_pts = np.array([
                            [-half,  half, 0],
                            [ half,  half, 0],
                            [ half, -half, 0],
                            [-half, -half, 0]
                        ], dtype=np.float32)
                        
                        success, rvec, tvec = cv2.solvePnP(
                            obj_pts, corners[i][0],
                            self.camera_matrix, self.dist_coeffs,
                            flags=cv2.SOLVEPNP_IPPE_SQUARE)
                        
                        if success:
                            # Visualization
                            self.latest_interaction_tf = (rvec, tvec)
                            
                            # Collection
                            if self.collecting:
                                self.stabilizer.add_sample(tvec, rvec)
                            return
            
            if not self.collecting:
                self.latest_interaction_tf = None
                
        except Exception as e:
            self.get_logger().warn(f'CV Error: {e}')

    def get_robot_marker_pose(self):
        """Get 'base_link' -> 'ee_aruco_marker' transform."""
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link', 'ee_aruco_marker', rclpy.time.Time())
            pos = t.transform.translation
            quat = t.transform.rotation
            T = np.eye(4)
            T[:3, :3] = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()
            T[:3, 3] = [pos.x, pos.y, pos.z]
            return T
        except:
            return None
    
    def get_camera_link_offset(self):
        """Get transform from camera_link to camera_color_optical_frame."""
        try:
            t = self.tf_buffer.lookup_transform(
                'camera_link', 'camera_color_optical_frame', rclpy.time.Time())
            pos = t.transform.translation
            quat = t.transform.rotation
            T = np.eye(4)
            T[:3, :3] = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()
            T[:3, 3] = [pos.x, pos.y, pos.z]
            return T
        except Exception as e:
            self.get_logger().warn(f'Could not get camera_link offset: {e}')
            return None

    def start_collection(self):
        self.stabilizer.clear()
        self.collecting = True
        self.get_logger().info('Collecting samples...')

    def process_buffer_and_add_sample(self):
        self.collecting = False
        
        tvec, quat, std_dev = self.stabilizer.get_stable_pose()
        if tvec is None:
            self.get_logger().warn('No samples collected (Marker not visible?)')
            return
            
        # Check stability
        stability_limit = 0.02  # 2cm
        jitter = np.linalg.norm(std_dev)
        
        self.get_logger().info(f'Jitter: {jitter*1000:.1f} mm')
        if jitter > stability_limit:
            self.get_logger().warn('High jitter! Camera or Robot moving?')
        
        # Build T_cam_marker (Stable)
        T_cam_marker = np.eye(4)
        T_cam_marker[:3, :3] = R.from_quat(quat).as_matrix()
        T_cam_marker[:3, 3] = tvec
        
        # Get Robot Pose
        T_base_marker = self.get_robot_marker_pose()
        if T_base_marker is None:
            self.get_logger().error('Could not get Robot TF!')
            return
            
        # Compute T_base_cam
        T_marker_cam = np.linalg.inv(T_cam_marker)
        T_base_cam = T_base_marker @ T_marker_cam
        
        self.calibration_samples.append(T_base_cam)
        
        pos = T_base_cam[:3, 3]
        self.get_logger().info(f'Sample Added. Camera Pos: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]')

    def compute_final_result(self):
        if not self.calibration_samples:
            return None
            
        # Robust average of final samples (Median again)
        positions = np.array([T[:3, 3] for T in self.calibration_samples])
        median_pos = np.median(positions, axis=0)
        
        quats = np.array([R.from_matrix(T[:3, :3]).as_quat() for T in self.calibration_samples])
        # Align quaternions for averaging
        ref_quat = quats[0]
        for i in range(len(quats)):
            if np.dot(quats[i], ref_quat) < 0:
                quats[i] = -quats[i]
                
        mean_quat = np.mean(quats, axis=0)
        mean_quat /= np.linalg.norm(mean_quat)
        
        T_final = np.eye(4)
        T_final[:3, :3] = R.from_quat(mean_quat).as_matrix()
        T_final[:3, 3] = median_pos
        
        return T_final

    def print_result(self, T_base_cam_optical):
        """Print and save calibration result for camera_link frame."""
        # Get the offset from camera_link to camera_color_optical_frame
        T_link_optical = self.get_camera_link_offset()
        
        if T_link_optical is not None:
            # Compute T_base_camera_link = T_base_cam_optical @ inv(T_link_optical)
            T_base_camera_link = T_base_cam_optical @ np.linalg.inv(T_link_optical)
            pos = T_base_camera_link[:3, 3]
            quat = R.from_matrix(T_base_camera_link[:3, :3]).as_quat()
            euler = R.from_matrix(T_base_camera_link[:3, :3]).as_euler('xyz', degrees=True)
            target_frame = 'camera_link'
        else:
            self.get_logger().warn('Could not compute camera_link offset, using camera_color_optical_frame')
            pos = T_base_cam_optical[:3, 3]
            quat = R.from_matrix(T_base_cam_optical[:3, :3]).as_quat()
            euler = R.from_matrix(T_base_cam_optical[:3, :3]).as_euler('xyz', degrees=True)
            target_frame = 'camera_color_optical_frame'
        
        msg = f'''
============================================================
FINAL CALIBRATION RESULT ({len(self.calibration_samples)} Samples)
============================================================
Target Frame: {target_frame}
Position (XYZ): [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}]
Euler (RPY):    [{euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}]
Quaternion:     [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]
============================================================
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_to_base_tf',
    arguments=[
        '{pos[0]:.6f}', '{pos[1]:.6f}', '{pos[2]:.6f}',
        '{quat[0]:.6f}', '{quat[1]:.6f}', '{quat[2]:.6f}', '{quat[3]:.6f}',
        'base_link', '{target_frame}'
    ]
)
'''
        self.get_logger().info(msg)
        
        # Save
        result = {
            'timestamp': datetime.now().isoformat(),
            'samples_count': len(self.calibration_samples),
            'target_frame': target_frame,
            'xyz': pos.tolist(),
            'xyzw': quat.tolist(),
            'rpy': euler.tolist()
        }
        with open('calibration_robust.json', 'a') as f:
            f.write(json.dumps(result) + '\n')

    def publish_viz(self):
        if self.latest_interaction_tf:
            rvec, tvec = self.latest_interaction_tf
            rmat, _ = cv2.Rodrigues(rvec)
            quat = R.from_matrix(rmat).as_quat()
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.camera_frame_id
            t.child_frame_id = f'aruco_marker_{self.marker_id}_live'
            
            t.transform.translation.x = float(tvec[0])
            t.transform.translation.y = float(tvec[1])
            t.transform.translation.z = float(tvec[2])
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = RobustCalibration()
    
    print("\nCorrected Eye-to-Hand Calibration")
    print("Commands:")
    print("  [ENTER] Collect Sample (takes ~2s)")
    print("  done    Compute Final Result")
    print("  q       Quit")
    
    # Single-threaded main loop logic
    try:
        while rclpy.ok():
            # Process ROS callbacks
            rclpy.spin_once(node, timeout_sec=0.01)
            node.publish_viz()
            
            # Check for collecting status
            if node.collecting:
                if node.stabilizer.is_full():
                    node.process_buffer_and_add_sample()
            
            # Non-blocking input check
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline().strip().lower()
                if cmd == 'q':
                    break
                elif cmd == 'done':
                    T = node.compute_final_result()
                    if T is not None:
                        node.print_result(T)
                    else:
                        print("No samples yet.")
                else:
                    if not node.collecting:
                        node.start_collection()
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Check if context is valid before shutdown
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
