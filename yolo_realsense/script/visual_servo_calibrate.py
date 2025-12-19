#!/usr/bin/env python3
"""
Visual Servoing Calibration Script for myCobot 320 M5 + RealSense D455

This script:
1. Uses visual servoing to align the ArUco marker PERFECTLY with the camera center
   (X=0, Y=0 in camera frame - only distance differs)
2. Once aligned, collects calibration samples by varying ONLY the distance
3. This gives much more accurate calibration since X/Y are locked

Usage:
    ros2 run yolo_realsense visual_servo_calibrate.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import json
from datetime import datetime


class VisualServoCalibrator(Node):
    def __init__(self):
        super().__init__('visual_servo_calibrator')
        
        # ------ Parameters ------
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.03)  # 30mm marker
        self.declare_parameter('num_samples', 12)
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('base_joint', -2.79)  # -160 degrees where camera is
        
        aruco_dict_name = self.get_parameter('aruco_dict').get_parameter_value().string_value
        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.num_samples = self.get_parameter('num_samples').get_parameter_value().integer_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.base_j1 = self.get_parameter('base_joint').get_parameter_value().double_value
        
        # ArUco dictionary
        aruco_dicts = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
        }
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dicts.get(aruco_dict_name, cv2.aruco.DICT_4X4_50))
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # ------ Camera ------
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_image = None
        self.image_width = 0
        self.image_height = 0
        
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        
        # ------ MoveIt ------
        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        self._exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        # Joint names for myCobot 320 M5
        self.joint_names = [
            'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
            'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'
        ]
        
        # Current joint state
        self.current_joints = [self.base_j1, -0.3, 0.3, 0.0, 0.0, 0.0]
        
        # Calibration data
        self.R_gripper2base_list = []
        self.t_gripper2base_list = []
        self.R_target2cam_list = []
        self.t_target2cam_list = []
        
        self.get_logger().info('Visual Servo Calibrator initialized')
        self.get_logger().info(f'ArUco: {aruco_dict_name}, ID: {self.marker_id}, Size: {self.marker_size}m')
    
    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_height, self.image_width = self.latest_image.shape[:2]
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
    
    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received')
    
    def detect_aruco(self):
        """Detect ArUco and return (center_x, center_y, rvec, tvec) or None."""
        if self.latest_image is None or self.camera_matrix is None:
            return None
        
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        
        if ids is None:
            return None
        
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == self.marker_id:
                marker_corners = corners[i][0]
                center_x = np.mean(marker_corners[:, 0])
                center_y = np.mean(marker_corners[:, 1])
                
                half_size = self.marker_size / 2.0
                obj_points = np.array([
                    [-half_size,  half_size, 0],
                    [ half_size,  half_size, 0],
                    [ half_size, -half_size, 0],
                    [-half_size, -half_size, 0]
                ], dtype=np.float32)
                
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, marker_corners,
                    self.camera_matrix, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                
                if success:
                    return center_x, center_y, rvec.flatten(), tvec.flatten()
        
        return None
    
    def move_to_joints(self, target_joints, wait_time=2.5):
        """Move robot to target joints."""
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            return False
        if not self._exec_client.wait_for_server(timeout_sec=5.0):
            return False
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        constraints = Constraints()
        for name, position in zip(self.joint_names, target_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = True
        
        send_goal_future = self._move_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.error_code.val != 1:
            return False
        
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = result.planned_trajectory
        
        exec_future = self._exec_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, exec_future)
        
        exec_handle = exec_future.result()
        if not exec_handle.accepted:
            return False
        
        exec_result_future = exec_handle.get_result_async()
        timeout = 30.0
        start = time.time()
        while not exec_result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                break
        
        self.current_joints = list(target_joints)
        time.sleep(wait_time)
        
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
        
        return True
    
    def visual_servo_to_center(self):
        """
        Use visual servoing to align marker PERFECTLY with camera center.
        Goal: marker tvec X ≈ 0, Y ≈ 0 (only Z distance differs)
        """
        self.get_logger().info('='*60)
        self.get_logger().info('PHASE 1: Visual Servoing to Center')
        self.get_logger().info('='*60)
        
        # Start position
        initial_pose = [self.base_j1, -0.3, 0.3, 0.0, 0.0, 0.0]
        self.get_logger().info(f'Moving to initial position...')
        self.move_to_joints(initial_pose, wait_time=3.0)
        
        # Visual servoing loop
        max_iterations = 30
        pixel_tolerance = 20  # pixels from center
        
        for iteration in range(max_iterations):
            detection = self.detect_aruco()
            
            if detection is None:
                self.get_logger().warn(f'Iteration {iteration+1}: Marker not visible, adjusting...')
                # Small random adjustment
                self.current_joints[0] += np.random.uniform(-0.1, 0.1)
                self.current_joints[1] += np.random.uniform(-0.05, 0.05)
                self.move_to_joints(self.current_joints, wait_time=2.0)
                continue
            
            center_x, center_y, rvec, tvec = detection
            
            # Calculate pixel error from image center
            error_px_x = center_x - self.image_width / 2
            error_px_y = center_y - self.image_height / 2
            
            # Calculate 3D position error (tvec X and Y should be ~0)
            error_3d_x = tvec[0]  # meters from camera center
            error_3d_y = tvec[1]  # meters from camera center
            distance = tvec[2]
            
            self.get_logger().info(
                f'Iter {iteration+1}: pixel_err=({error_px_x:.0f}, {error_px_y:.0f}), '
                f'3D_err=({error_3d_x*1000:.1f}, {error_3d_y*1000:.1f})mm, dist={distance:.3f}m'
            )
            
            # Check if aligned
            if abs(error_px_x) < pixel_tolerance and abs(error_px_y) < pixel_tolerance:
                self.get_logger().info('✓ ALIGNED! Marker centered in camera view.')
                self.get_logger().info(f'  Final 3D offset: X={error_3d_x*1000:.2f}mm, Y={error_3d_y*1000:.2f}mm')
                return True
            
            # Visual servoing gains
            # Use PIXEL error since it's more reliable than 3D position
            # Smaller gains for stability
            gain_j1 = 0.0002  # For horizontal pixel error -> joint 1 rotation
            gain_j2 = 0.0002  # For vertical pixel error -> joint 2 (shoulder)
            gain_j3 = 0.0001  # For vertical pixel error -> joint 3 (elbow)
            
            # Compute joint adjustments based on PIXEL error
            # When marker is to the LEFT of center (negative error_px_x), 
            # we need to rotate joint 1 NEGATIVE to move marker RIGHT
            delta_j1 = error_px_x * gain_j1  # Changed sign
            
            # When marker is ABOVE center (negative error_px_y),
            # we need to move arm DOWN to bring marker DOWN in image
            delta_j2 = -error_px_y * gain_j2  # Negative error = up, so negate
            delta_j3 = error_px_y * gain_j3   
            
            # Clamp adjustments to small values for stability
            delta_j1 = np.clip(delta_j1, -0.05, 0.05)
            delta_j2 = np.clip(delta_j2, -0.04, 0.04)
            delta_j3 = np.clip(delta_j3, -0.03, 0.03)
            
            # Apply
            new_joints = self.current_joints.copy()
            new_joints[0] += delta_j1
            new_joints[1] += delta_j2
            new_joints[2] += delta_j3
            
            # Clamp to limits
            new_joints[0] = np.clip(new_joints[0], self.base_j1 - 1.0, self.base_j1 + 1.0)
            new_joints[1] = np.clip(new_joints[1], -1.2, 0.8)
            new_joints[2] = np.clip(new_joints[2], -0.8, 1.2)
            
            self.get_logger().info(f'  Adjusting: j1+={delta_j1:.4f}, j2+={delta_j2:.4f}, j3+={delta_j3:.4f}')
            self.move_to_joints(new_joints, wait_time=2.0)
        
        # Check final state
        detection = self.detect_aruco()
        if detection is not None:
            self.get_logger().warn('Max iterations reached, but marker visible. Proceeding.')
            return True
        
        return False
    
    def compute_fk(self, joints):
        """Compute forward kinematics."""
        dh_params = [
            [0.1315, 0.0, np.pi/2],
            [0.0, -0.1104, 0.0],
            [0.0, -0.096, 0.0],
            [0.0631, 0.0, np.pi/2],
            [0.0735, 0.0, -np.pi/2],
            [0.0456, 0.0, 0.0],
        ]
        
        def dh_matrix(theta, d, a, alpha):
            ct, st = np.cos(theta), np.sin(theta)
            ca, sa = np.cos(alpha), np.sin(alpha)
            return np.array([
                [ct, -st*ca, st*sa, a*ct],
                [st, ct*ca, -ct*sa, a*st],
                [0, sa, ca, d],
                [0, 0, 0, 1]
            ])
        
        T = np.eye(4)
        for params, theta in zip(dh_params, joints):
            d, a, alpha = params
            T = T @ dh_matrix(theta, d, a, alpha)
        
        return T[:3, :3], T[:3, 3]
    
    def collect_samples_varying_distance(self):
        """
        Collect calibration samples by varying ONLY the distance.
        Keep X/Y aligned, move closer/farther from camera.
        """
        self.get_logger().info('='*60)
        self.get_logger().info('PHASE 2: Collecting Samples (Varying Distance Only)')
        self.get_logger().info('='*60)
        
        base_pose = self.current_joints.copy()
        
        # Variations that change distance without changing XY alignment much
        # Joint 2 and 3 together move the arm closer/farther
        distance_variations = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],        # Current centered position
            [0.0, -0.05, 0.05, 0.0, 0.0, 0.0],    # Slightly farther
            [0.0, 0.05, -0.05, 0.0, 0.0, 0.0],    # Slightly closer
            [0.0, -0.1, 0.1, 0.0, 0.0, 0.0],      # Farther
            [0.0, 0.1, -0.1, 0.0, 0.0, 0.0],      # Closer
            [0.0, -0.15, 0.15, 0.0, 0.0, 0.0],    # More farther
            [0.0, 0.15, -0.15, 0.0, 0.0, 0.0],    # More closer
            # Also add some wrist rotations (changes orientation but keeps XY)
            [0.0, 0.0, 0.0, 0.2, 0.0, 0.0],
            [0.0, 0.0, 0.0, -0.2, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.2, 0.0],
            [0.0, 0.0, 0.0, 0.0, -0.2, 0.0],
            [0.0, -0.08, 0.08, 0.15, 0.0, 0.0],   # Combined
        ]
        
        samples_collected = 0
        
        for i, var in enumerate(distance_variations[:self.num_samples]):
            self.get_logger().info(f'\n--- Sample {i+1}/{min(len(distance_variations), self.num_samples)} ---')
            
            target = [base_pose[j] + var[j] for j in range(6)]
            
            self.get_logger().info(f'Variation: {[round(v, 2) for v in var]}')
            if not self.move_to_joints(target, wait_time=2.5):
                self.get_logger().warn(f'Failed to move')
                continue
            
            detection = self.detect_aruco()
            if detection is None:
                self.get_logger().warn(f'Marker not detected, skipping')
                continue
            
            center_x, center_y, rvec, tvec = detection
            self.get_logger().info(f'✓ Detected: tvec=({tvec[0]*1000:.1f}, {tvec[1]*1000:.1f}, {tvec[2]*1000:.1f})mm')
            
            # Get FK
            R_base_ee, t_base_ee = self.compute_fk(target)
            
            R_ee2base = R_base_ee.T
            t_ee2base = -R_base_ee.T @ t_base_ee
            
            R_marker2cam, _ = cv2.Rodrigues(rvec)
            t_marker2cam = tvec
            
            self.R_gripper2base_list.append(R_ee2base)
            self.t_gripper2base_list.append(t_ee2base.reshape(3, 1))
            self.R_target2cam_list.append(R_marker2cam)
            self.t_target2cam_list.append(t_marker2cam.reshape(3, 1))
            
            samples_collected += 1
            self.get_logger().info(f'Sample {samples_collected} collected.')
        
        return samples_collected
    
    def solve_calibration(self, samples_collected):
        """Solve hand-eye calibration and output results."""
        if samples_collected < 4:
            self.get_logger().error(f'Only {samples_collected} samples. Need ≥4.')
            return
        
        self.get_logger().info(f'\nComputing calibration with {samples_collected} samples...')
        
        R_cam2base, t_cam2base = cv2.calibrateHandEye(
            self.R_gripper2base_list,
            self.t_gripper2base_list,
            self.R_target2cam_list,
            self.t_target2cam_list,
            method=cv2.CALIB_HAND_EYE_TSAI
        )
        
        T_cam2base = np.eye(4)
        T_cam2base[:3, :3] = R_cam2base
        T_cam2base[:3, 3] = t_cam2base.flatten()
        
        pos = t_cam2base.flatten()
        distance = np.linalg.norm(pos)
        
        r = R.from_matrix(R_cam2base)
        euler = r.as_euler('xyz', degrees=True)
        quat = r.as_quat()
        
        self.get_logger().info('='*60)
        self.get_logger().info('CALIBRATION RESULTS')
        self.get_logger().info('='*60)
        self.get_logger().info(f'\nT_camera_to_base:')
        for row in T_cam2base:
            self.get_logger().info(f'  [{row[0]:+.6f}, {row[1]:+.6f}, {row[2]:+.6f}, {row[3]:+.6f}]')
        
        self.get_logger().info(f'\nCamera position (m): X={pos[0]:+.4f}, Y={pos[1]:+.4f}, Z={pos[2]:+.4f}')
        self.get_logger().info(f'Euler (deg): Roll={euler[0]:+.2f}, Pitch={euler[1]:+.2f}, Yaw={euler[2]:+.2f}')
        self.get_logger().info(f'\n*** DISTANCE: {distance:.4f} m ({distance*1000:.2f} mm) ***')
        
        result = {
            'timestamp': datetime.now().isoformat(),
            'samples': samples_collected,
            'T_camera_to_base': T_cam2base.tolist(),
            'position': pos.tolist(),
            'euler_xyz_deg': euler.tolist(),
            'quaternion_xyzw': quat.tolist(),
            'distance_m': float(distance),
        }
        
        filename = f'servo_calibration_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        with open(filename, 'w') as f:
            json.dump(result, f, indent=2)
        self.get_logger().info(f'\nSaved: {filename}')
        
        self.get_logger().info(f'''
--- Static TF Publisher ---
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['{pos[0]:.6f}', '{pos[1]:.6f}', '{pos[2]:.6f}',
               '{quat[0]:.6f}', '{quat[1]:.6f}', '{quat[2]:.6f}', '{quat[3]:.6f}',
               'base_link', 'camera_link']
)
''')
        
        self.get_logger().info('='*60)
        self.get_logger().info('Calibration complete!')
        self.get_logger().info('='*60)
    
    def run(self):
        """Main execution."""
        self.get_logger().info('Waiting for camera...')
        timeout = 30.0
        start = time.time()
        while self.camera_matrix is None or self.latest_image is None:
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - start > timeout:
                self.get_logger().error('Camera not available!')
                return
        self.get_logger().info('Camera ready.')
        
        # Phase 1: Visual servo to align XY
        if not self.visual_servo_to_center():
            self.get_logger().error('Visual servoing failed!')
            return
        
        # Phase 2: Collect samples varying distance
        samples = self.collect_samples_varying_distance()
        
        # Phase 3: Solve calibration
        self.solve_calibration(samples)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoCalibrator()
    
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
