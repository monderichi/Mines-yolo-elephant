#!/usr/bin/env python3
"""
Eye-to-Hand Calibration for myCobot 320 M5 + RealSense D455

This script performs eye-to-hand calibration using an ArUco marker
attached to the robot's end effector. The camera is stationary.

Features:
- ArUco marker detection (ID 0, 30mm, DICT_4X4_50)
- TF broadcasting for visualization in RViz
- Automated calibration sample collection
- Hand-eye calibration using OpenCV

Usage:
    ros2 run yolo_realsense eye_to_hand_calibration.py

Author: Generated for myCobot 320 M5
"""

import rclpy
from rclpy.node import Node
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
import threading


class EyeToHandCalibration(Node):
    def __init__(self):
        super().__init__('eye_to_hand_calibration')
        
        # ------ Parameters ------
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.030)  # 30mm
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('num_samples', 15)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'ee_aruco_marker')  # Frame where marker is attached
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        
        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        aruco_dict_name = self.get_parameter('aruco_dict').get_parameter_value().string_value
        self.num_samples = self.get_parameter('num_samples').get_parameter_value().integer_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.ee_frame = self.get_parameter('ee_frame').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        
        # ------ ArUco Setup ------
        aruco_dicts = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
        }
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            aruco_dicts.get(aruco_dict_name, cv2.aruco.DICT_4X4_50))
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # ------ Camera ------
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_image = None
        self.camera_frame_id = 'camera_color_optical_frame'
        
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        
        # ------ TF ------
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ------ Calibration Data ------
        self.R_gripper2base_list = []
        self.t_gripper2base_list = []
        self.R_target2cam_list = []
        self.t_target2cam_list = []
        self.ee_positions = []  # Track EE positions for diversity check
        
        # Current marker detection
        self.current_marker_pose = None  # (rvec, tvec)
        self.marker_detected = False
        
        # State
        self.calibration_complete = False
        self.T_cam2base = None
        
        # Pose diversity threshold (meters) - reject poses closer than this
        self.min_pose_distance = 0.03  # 3cm minimum movement
        
        # Timer for continuous ArUco broadcasting
        self.create_timer(0.05, self.broadcast_aruco_tf)  # 20Hz
        
        self.get_logger().info('='*60)
        self.get_logger().info('Eye-to-Hand Calibration Node Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Marker ID: {self.marker_id}, Size: {self.marker_size*1000:.0f}mm')
        self.get_logger().info(f'Base frame: {self.base_frame}, EE frame: {self.ee_frame}')
        self.get_logger().info('='*60)

    def camera_info_callback(self, msg):
        """Extract camera intrinsics."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_frame_id = msg.header.frame_id
            self.get_logger().info(f'Camera intrinsics received (frame: {self.camera_frame_id})')

    def image_callback(self, msg):
        """Process incoming images and detect ArUco markers."""
        if self.camera_matrix is None:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_image = cv_image
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
            
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id == self.marker_id:
                        # Estimate pose
                        half_size = self.marker_size / 2.0
                        obj_points = np.array([
                            [-half_size,  half_size, 0],
                            [ half_size,  half_size, 0],
                            [ half_size, -half_size, 0],
                            [-half_size, -half_size, 0]
                        ], dtype=np.float32)
                        
                        success, rvec, tvec = cv2.solvePnP(
                            obj_points, corners[i][0],
                            self.camera_matrix, self.dist_coeffs,
                            flags=cv2.SOLVEPNP_IPPE_SQUARE
                        )
                        
                        if success:
                            self.current_marker_pose = (rvec.flatten(), tvec.flatten())
                            self.marker_detected = True
                            return
            
            self.marker_detected = False
            self.current_marker_pose = None
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def broadcast_aruco_tf(self):
        """Broadcast ArUco marker TF for visualization."""
        if self.current_marker_pose is None:
            return
            
        rvec, tvec = self.current_marker_pose
        
        # Create transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame_id
        t.child_frame_id = f'aruco_marker_{self.marker_id}'
        
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        
        rmat, _ = cv2.Rodrigues(rvec)
        quat = R.from_matrix(rmat).as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)

    def get_ee_pose(self):
        """Get end effector pose in base frame from TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Extract translation
            t = transform.transform.translation
            translation = np.array([t.x, t.y, t.z])
            
            # Extract rotation
            q = transform.transform.rotation
            rotation = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            
            return rotation, translation
            
        except Exception as e:
            self.get_logger().warn(f'Could not get EE pose: {e}')
            return None, None

    def check_pose_diversity(self, new_pos):
        """Check if the new pose is different enough from previous samples."""
        if len(self.ee_positions) == 0:
            return True
        
        for prev_pos in self.ee_positions:
            distance = np.linalg.norm(new_pos - prev_pos)
            if distance < self.min_pose_distance:
                return False
        return True

    def collect_sample(self):
        """Collect a single calibration sample."""
        if not self.marker_detected or self.current_marker_pose is None:
            self.get_logger().warn('ArUco marker not detected!')
            return False
        
        # Get EE pose from TF
        R_base_ee, t_base_ee = self.get_ee_pose()
        if R_base_ee is None:
            self.get_logger().warn('Could not get robot pose from TF!')
            return False
        
        # Check if pose is different enough from previous samples
        if not self.check_pose_diversity(t_base_ee):
            self.get_logger().warn('⚠ Pose too similar to previous sample!')
            self.get_logger().warn('  >>> MOVE THE ROBOT to a different position before capturing! <<<')
            return False
        
        # Get marker pose in camera
        rvec, tvec = self.current_marker_pose
        R_cam_marker, _ = cv2.Rodrigues(rvec)
        t_cam_marker = tvec
        
        # For eye-to-hand: we need T_gripper2base (inverse of T_base2gripper)
        # T_base2gripper = (R_base_ee, t_base_ee)
        # T_gripper2base = inv(T_base2gripper)
        R_ee2base = R_base_ee.T
        t_ee2base = -R_base_ee.T @ t_base_ee
        
        # Store samples
        self.R_gripper2base_list.append(R_ee2base)
        self.t_gripper2base_list.append(t_ee2base.reshape(3, 1))
        self.R_target2cam_list.append(R_cam_marker)
        self.t_target2cam_list.append(t_cam_marker.reshape(3, 1))
        self.ee_positions.append(t_base_ee.copy())
        
        sample_num = len(self.R_gripper2base_list)
        self.get_logger().info(f'✓ Sample {sample_num} collected')
        self.get_logger().info(f'  EE position: [{t_base_ee[0]:.3f}, {t_base_ee[1]:.3f}, {t_base_ee[2]:.3f}]')
        self.get_logger().info(f'  Marker in cam: [{tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}]')
        
        return True

    def compute_calibration(self):
        """Compute the hand-eye calibration."""
        n_samples = len(self.R_gripper2base_list)
        
        if n_samples < 4:
            self.get_logger().error(f'Need at least 4 samples, only have {n_samples}')
            return False
        
        self.get_logger().info(f'\nComputing calibration with {n_samples} samples...')
        
        try:
            # Eye-to-hand calibration
            R_cam2base, t_cam2base = cv2.calibrateHandEye(
                self.R_gripper2base_list,
                self.t_gripper2base_list,
                self.R_target2cam_list,
                self.t_target2cam_list,
                method=cv2.CALIB_HAND_EYE_TSAI
            )
            
            # Build 4x4 transformation matrix
            self.T_cam2base = np.eye(4)
            self.T_cam2base[:3, :3] = R_cam2base
            self.T_cam2base[:3, 3] = t_cam2base.flatten()
            
            self.calibration_complete = True
            return True
            
        except Exception as e:
            self.get_logger().error(f'Calibration failed: {e}')
            return False

    def print_results(self):
        """Print calibration results."""
        if self.T_cam2base is None:
            return
        
        cam_pos = self.T_cam2base[:3, 3]
        R_cam2base = self.T_cam2base[:3, :3]
        distance = np.linalg.norm(cam_pos)
        
        r = R.from_matrix(R_cam2base)
        euler = r.as_euler('xyz', degrees=True)
        quat = r.as_quat()  # [x, y, z, w]
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('CALIBRATION RESULTS')
        self.get_logger().info('='*60)
        
        self.get_logger().info('\nT_camera_to_base (4x4 matrix):')
        for row in self.T_cam2base:
            self.get_logger().info(f'  [{row[0]:+.6f}, {row[1]:+.6f}, {row[2]:+.6f}, {row[3]:+.6f}]')
        
        self.get_logger().info(f'\nCamera position in base frame (meters):')
        self.get_logger().info(f'  X: {cam_pos[0]:+.4f}')
        self.get_logger().info(f'  Y: {cam_pos[1]:+.4f}')
        self.get_logger().info(f'  Z: {cam_pos[2]:+.4f}')
        self.get_logger().info(f'  Distance: {distance:.4f} m ({distance*1000:.1f} mm)')
        
        self.get_logger().info(f'\nCamera orientation (Euler XYZ, degrees):')
        self.get_logger().info(f'  Roll:  {euler[0]:+.2f}°')
        self.get_logger().info(f'  Pitch: {euler[1]:+.2f}°')
        self.get_logger().info(f'  Yaw:   {euler[2]:+.2f}°')
        
        self.get_logger().info(f'\nQuaternion [x, y, z, w]:')
        self.get_logger().info(f'  [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]')
        
        self.get_logger().info('\n' + '-'*60)
        self.get_logger().info('STATIC TRANSFORM PUBLISHER COMMAND:')
        self.get_logger().info('-'*60)
        self.get_logger().info(f'''
ros2 run tf2_ros static_transform_publisher \\
    --x {cam_pos[0]:.6f} --y {cam_pos[1]:.6f} --z {cam_pos[2]:.6f} \\
    --qx {quat[0]:.6f} --qy {quat[1]:.6f} --qz {quat[2]:.6f} --qw {quat[3]:.6f} \\
    --frame-id {self.base_frame} --child-frame-id {self.camera_frame_id}
''')
        
        self.get_logger().info('\n' + '-'*60)
        self.get_logger().info('FOR LAUNCH FILE:')
        self.get_logger().info('-'*60)
        self.get_logger().info(f'''
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_to_base_tf',
    arguments=[
        '{cam_pos[0]:.6f}', '{cam_pos[1]:.6f}', '{cam_pos[2]:.6f}',
        '{quat[0]:.6f}', '{quat[1]:.6f}', '{quat[2]:.6f}', '{quat[3]:.6f}',
        '{self.base_frame}', '{self.camera_frame_id}'
    ]
)
''')
        self.get_logger().info('='*60)

    def save_results(self):
        """Save calibration results to JSON file."""
        if self.T_cam2base is None:
            return
        
        cam_pos = self.T_cam2base[:3, 3]
        R_cam2base = self.T_cam2base[:3, :3]
        
        r = R.from_matrix(R_cam2base)
        euler = r.as_euler('xyz', degrees=True)
        quat = r.as_quat()
        
        result = {
            'timestamp': datetime.now().isoformat(),
            'num_samples': len(self.R_gripper2base_list),
            'T_camera_to_base': self.T_cam2base.tolist(),
            'camera_position_xyz': cam_pos.tolist(),
            'camera_orientation_euler_xyz_deg': euler.tolist(),
            'camera_orientation_quaternion_xyzw': quat.tolist(),
            'distance_m': float(np.linalg.norm(cam_pos)),
            'base_frame': self.base_frame,
            'camera_frame': self.camera_frame_id,
        }
        
        filename = f'eye_to_hand_calibration_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        with open(filename, 'w') as f:
            json.dump(result, f, indent=2)
        self.get_logger().info(f'\nResults saved to: {filename}')

    def broadcast_calibrated_camera_tf(self):
        """Broadcast the calibrated camera transform statically."""
        if self.T_cam2base is None:
            return
        
        cam_pos = self.T_cam2base[:3, 3]
        R_cam2base = self.T_cam2base[:3, :3]
        quat = R.from_matrix(R_cam2base).as_quat()
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.camera_frame_id
        
        t.transform.translation.x = float(cam_pos[0])
        t.transform.translation.y = float(cam_pos[1])
        t.transform.translation.z = float(cam_pos[2])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.static_broadcaster.sendTransform(t)
        self.get_logger().info(f'Broadcasting calibrated camera TF: {self.base_frame} -> {self.camera_frame_id}')

    def run_interactive_calibration(self):
        """Run interactive calibration mode."""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('INTERACTIVE CALIBRATION MODE')
        self.get_logger().info('='*60)
        self.get_logger().info('''
╔══════════════════════════════════════════════════════════╗
║  IMPORTANT: You must MOVE THE ROBOT between each sample! ║
╚══════════════════════════════════════════════════════════╝

Instructions:
  1. Use MoveIt/RViz to move the robot arm to a NEW position
  2. Ensure the ArUco marker is visible to the camera
  3. Press ENTER to capture a sample
  4. MOVE THE ROBOT AGAIN to a DIFFERENT position
  5. Repeat for {} samples (minimum 4, include rotations!)
  6. Type 'done' to compute calibration
  7. Type 'clear' to restart sample collection
  8. Type 'quit' to exit

Tips for good calibration:
  - Include different joint rotations, not just translations
  - Try poses with the wrist rotated differently
  - Spread poses across the robot's workspace
'''.format(self.num_samples))
        
        # Wait for camera
        self.get_logger().info('Waiting for camera...')
        timeout = 30.0
        start = time.time()
        while self.camera_matrix is None:
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - start > timeout:
                self.get_logger().error('Camera not available!')
                return
        self.get_logger().info('Camera ready!')
        
        # Wait for TF
        self.get_logger().info('Waiting for TF...')
        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=0.5)
        
        # Input handling in separate thread
        user_input = [None]
        input_ready = threading.Event()
        
        def get_input():
            while rclpy.ok():
                try:
                    user_input[0] = input('\n[Press ENTER to capture, or type command]: ').strip().lower()
                    input_ready.set()
                except EOFError:
                    break
        
        input_thread = threading.Thread(target=get_input, daemon=True)
        input_thread.start()
        
        while rclpy.ok():
            # Spin to process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Check for user input
            if input_ready.is_set():
                cmd = user_input[0]
                input_ready.clear()
                
                if cmd == 'quit' or cmd == 'q':
                    self.get_logger().info('Exiting...')
                    break
                    
                elif cmd == 'done' or cmd == 'd':
                    if self.compute_calibration():
                        self.print_results()
                        self.save_results()
                        self.broadcast_calibrated_camera_tf()
                        self.get_logger().info('\nCalibration complete! Camera TF is now being broadcast.')
                        self.get_logger().info('Keep this node running to maintain the TF, or copy the command above.')
                        
                        # Keep running to broadcast TF
                        while rclpy.ok():
                            rclpy.spin_once(self, timeout_sec=0.5)
                    
                elif cmd == 'clear' or cmd == 'c':
                    self.R_gripper2base_list.clear()
                    self.t_gripper2base_list.clear()
                    self.R_target2cam_list.clear()
                    self.t_target2cam_list.clear()
                    self.ee_positions.clear()
                    self.get_logger().info('Samples cleared.')
                    
                elif cmd == 'status' or cmd == 's':
                    n = len(self.R_gripper2base_list)
                    marker_status = '✓ DETECTED' if self.marker_detected else '✗ NOT DETECTED'
                    self.get_logger().info(f'Samples: {n}/{self.num_samples}, Marker: {marker_status}')
                    
                else:  # Empty or unknown - try to capture
                    if self.collect_sample():
                        n = len(self.R_gripper2base_list)
                        if n >= self.num_samples:
                            self.get_logger().info(f'\nReached {self.num_samples} samples. Type "done" to compute calibration.')


def main(args=None):
    rclpy.init(args=args)
    node = EyeToHandCalibration()
    
    try:
        node.run_interactive_calibration()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
