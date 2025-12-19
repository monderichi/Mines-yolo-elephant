#!/usr/bin/env python3
"""
ArUco Alignment and Calibration Script for myCobot 320 M5 + RealSense D455

This script:
1. First aligns the ArUco marker to be centered in the camera view
2. Uses visual servoing to keep the marker visible
3. Then runs the full hand-eye calibration

Usage:
    ros2 run yolo_realsense align_and_calibrate.py
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


class AlignAndCalibrate(Node):
    def __init__(self):
        super().__init__('align_and_calibrate')
        
        # ------ Parameters ------
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.03)  # 30mm marker
        self.declare_parameter('num_samples', 15)
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
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
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
        
        # Current joint state (will be updated)
        self.current_joints = [self.base_j1, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Collected calibration data
        self.R_gripper2base_list = []
        self.t_gripper2base_list = []
        self.R_target2cam_list = []
        self.t_target2cam_list = []
        
        self.get_logger().info('Align and Calibrate initialized')
        self.get_logger().info(f'ArUco dict: {aruco_dict_name}, Marker ID: {self.marker_id}, Size: {self.marker_size}m')
    
    def image_callback(self, msg):
        """Store latest image."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_height, self.image_width = self.latest_image.shape[:2]
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
    
    def camera_info_callback(self, msg):
        """Extract camera intrinsics."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received')
    
    def detect_aruco(self):
        """Detect ArUco marker and return (center_x, center_y, rvec, tvec) or None."""
        if self.latest_image is None or self.camera_matrix is None:
            return None
        
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        
        if ids is None:
            return None
        
        # Find our marker
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == self.marker_id:
                # Get marker center
                marker_corners = corners[i][0]
                center_x = np.mean(marker_corners[:, 0])
                center_y = np.mean(marker_corners[:, 1])
                
                # Estimate pose
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
    
    def move_to_joints(self, target_joints, wait_time=3.0):
        """Move robot to target joint configuration. Returns True on success."""
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available!')
            return False
        if not self._exec_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('ExecuteTrajectory action server not available!')
            return False
        
        # Create motion plan request
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
        
        # Plan
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
        
        # Execute
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = result.planned_trajectory
        
        exec_future = self._exec_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, exec_future)
        
        exec_handle = exec_future.result()
        if not exec_handle.accepted:
            return False
        
        # Wait for execution
        exec_result_future = exec_handle.get_result_async()
        timeout = 30.0
        start = time.time()
        while not exec_result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                break
        
        # Update current joints
        self.current_joints = list(target_joints)
        
        # Wait for robot to settle
        time.sleep(wait_time)
        
        # Spin to get latest image
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
        
        return True
    
    def align_marker_to_center(self):
        """
        Move robot to find ArUco marker in camera view.
        Returns True if marker is found.
        """
        self.get_logger().info('='*60)
        self.get_logger().info('PHASE 1: Finding ArUco marker')
        self.get_logger().info('='*60)
        
        # Start with base position facing camera
        self.get_logger().info(f'Moving to initial position facing camera (joint1 = {self.base_j1:.2f} rad)')
        initial_pose = [self.base_j1, -0.3, 0.3, 0.0, 0.0, 0.0]
        self.move_to_joints(initial_pose, wait_time=3.0)
        self.current_joints = initial_pose.copy()
        
        # Check if we can see the marker
        detection = self.detect_aruco()
        if detection is not None:
            center_x, center_y, rvec, tvec = detection
            self.get_logger().info(f'✓ Marker found! Position: ({center_x:.0f}, {center_y:.0f}), distance: {tvec[2]:.3f}m')
            return True
        
        # Search for the marker by trying different positions
        self.get_logger().info('Searching for marker...')
        search_poses = [
            [self.base_j1, -0.2, 0.2, 0.0, 0.0, 0.0],
            [self.base_j1, -0.4, 0.4, 0.0, 0.0, 0.0],
            [self.base_j1 + 0.2, -0.3, 0.3, 0.0, 0.0, 0.0],
            [self.base_j1 - 0.2, -0.3, 0.3, 0.0, 0.0, 0.0],
            [self.base_j1, -0.5, 0.5, 0.0, 0.0, 0.0],
        ]
        
        for i, pose in enumerate(search_poses):
            self.get_logger().info(f'Search pose {i+1}/{len(search_poses)}')
            self.move_to_joints(pose, wait_time=2.5)
            self.current_joints = pose.copy()
            
            detection = self.detect_aruco()
            if detection is not None:
                center_x, center_y, rvec, tvec = detection
                self.get_logger().info(f'✓ Marker found! Position: ({center_x:.0f}, {center_y:.0f}), distance: {tvec[2]:.3f}m')
                return True
        
        self.get_logger().error('Could not find ArUco marker!')
        return False
    
    def compute_fk(self, joints):
        """Compute forward kinematics for myCobot 320 M5."""
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
        for i, (params, theta) in enumerate(zip(dh_params, joints)):
            d, a, alpha = params
            T = T @ dh_matrix(theta, d, a, alpha)
        
        return T[:3, :3], T[:3, 3]
    
    def run_calibration(self):
        """Run calibration with poses that keep marker visible."""
        self.get_logger().info('='*60)
        self.get_logger().info('PHASE 2: Collecting calibration samples')
        self.get_logger().info('='*60)
        
        # Generate poses around current centered position
        base_pose = self.current_joints.copy()
        
        # Small variations that should keep marker visible
        variations = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],      # Current centered position
            [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],      # Slight rotation
            [-0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, -0.1, 0.1, 0.0, 0.0, 0.0],     # Move arm down/up
            [0.0, 0.1, -0.1, 0.0, 0.0, 0.0],
            [0.15, -0.05, 0.05, 0.0, 0.0, 0.0],  # Combined small movements
            [-0.15, -0.05, 0.05, 0.0, 0.0, 0.0],
            [0.0, -0.15, 0.15, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.2, 0.0, 0.0],      # Wrist rotation
            [0.0, 0.0, 0.0, -0.2, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.3, 0.0],      # Wrist tilt
            [0.0, 0.0, 0.0, 0.0, -0.3, 0.0],
            [0.08, -0.08, 0.08, 0.15, 0.0, 0.0], # Combined
            [-0.08, -0.08, 0.08, -0.15, 0.0, 0.0],
            [0.0, -0.2, 0.2, 0.0, 0.0, 0.0],
        ]
        
        samples_collected = 0
        
        for i, var in enumerate(variations[:self.num_samples]):
            self.get_logger().info(f'\n--- Sample {i+1}/{min(len(variations), self.num_samples)} ---')
            
            # Apply variation to base pose
            target = [base_pose[j] + var[j] for j in range(6)]
            
            self.get_logger().info(f'Moving to variation: {[round(v, 2) for v in var]}')
            if not self.move_to_joints(target, wait_time=2.5):
                self.get_logger().warn(f'Failed to move to pose {i+1}')
                continue
            
            # Detect ArUco
            detection = self.detect_aruco()
            if detection is None:
                self.get_logger().warn(f'ArUco not detected at pose {i+1}, skipping')
                continue
            
            center_x, center_y, rvec, tvec = detection
            self.get_logger().info(f'ArUco detected! tvec={tvec}, center=({center_x:.0f}, {center_y:.0f})')
            
            # Get FK
            R_base_ee, t_base_ee = self.compute_fk(target)
            
            # For eye-to-hand: T_gripper2base = inv(T_base_gripper)
            R_ee2base = R_base_ee.T
            t_ee2base = -R_base_ee.T @ t_base_ee
            
            # T_target2cam from ArUco
            R_marker2cam, _ = cv2.Rodrigues(rvec)
            t_marker2cam = tvec
            
            # Store
            self.R_gripper2base_list.append(R_ee2base)
            self.t_gripper2base_list.append(t_ee2base.reshape(3, 1))
            self.R_target2cam_list.append(R_marker2cam)
            self.t_target2cam_list.append(t_marker2cam.reshape(3, 1))
            
            samples_collected += 1
            self.get_logger().info(f'Sample {samples_collected} collected.')
        
        if samples_collected < 4:
            self.get_logger().error(f'Only {samples_collected} samples. Need at least 4.')
            return
        
        self.get_logger().info(f'\nCollected {samples_collected} samples. Computing calibration...')
        
        # Solve hand-eye calibration
        R_cam2base, t_cam2base = cv2.calibrateHandEye(
            self.R_gripper2base_list,
            self.t_gripper2base_list,
            self.R_target2cam_list,
            self.t_target2cam_list,
            method=cv2.CALIB_HAND_EYE_TSAI
        )
        
        # Build 4x4 transformation
        T_cam2base = np.eye(4)
        T_cam2base[:3, :3] = R_cam2base
        T_cam2base[:3, 3] = t_cam2base.flatten()
        
        camera_position = t_cam2base.flatten()
        distance = np.linalg.norm(camera_position)
        
        r = R.from_matrix(R_cam2base)
        euler = r.as_euler('xyz', degrees=True)
        quat = r.as_quat()
        
        # Print results
        self.get_logger().info('='*60)
        self.get_logger().info('CALIBRATION RESULTS')
        self.get_logger().info('='*60)
        self.get_logger().info(f'\nT_camera_to_base (4x4 matrix):')
        for row in T_cam2base:
            self.get_logger().info(f'  [{row[0]:+.6f}, {row[1]:+.6f}, {row[2]:+.6f}, {row[3]:+.6f}]')
        
        self.get_logger().info(f'\nCamera position in base frame (meters):')
        self.get_logger().info(f'  X: {camera_position[0]:+.4f}')
        self.get_logger().info(f'  Y: {camera_position[1]:+.4f}')
        self.get_logger().info(f'  Z: {camera_position[2]:+.4f}')
        
        self.get_logger().info(f'\nCamera orientation (Euler XYZ, degrees):')
        self.get_logger().info(f'  Roll:  {euler[0]:+.2f}°')
        self.get_logger().info(f'  Pitch: {euler[1]:+.2f}°')
        self.get_logger().info(f'  Yaw:   {euler[2]:+.2f}°')
        
        self.get_logger().info(f'\n*** Distance: {distance:.4f} m ({distance*1000:.2f} mm) ***')
        
        # Save
        result = {
            'timestamp': datetime.now().isoformat(),
            'samples_used': samples_collected,
            'T_camera_to_base': T_cam2base.tolist(),
            'camera_position_in_base': camera_position.tolist(),
            'camera_orientation_euler_xyz_deg': euler.tolist(),
            'camera_orientation_quaternion_xyzw': quat.tolist(),
            'distance_m': float(distance),
        }
        
        filename = f'aligned_calibration_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        with open(filename, 'w') as f:
            json.dump(result, f, indent=2)
        self.get_logger().info(f'\nResults saved to: {filename}')
        
        # TF command
        self.get_logger().info('\n--- Static Transform Publisher ---')
        self.get_logger().info(f'''
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_to_base_tf',
    arguments=[
        '{camera_position[0]:.6f}', '{camera_position[1]:.6f}', '{camera_position[2]:.6f}',
        '{quat[0]:.6f}', '{quat[1]:.6f}', '{quat[2]:.6f}', '{quat[3]:.6f}',
        'base_link', 'camera_link'
    ]
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
        
        # Phase 1: Align marker
        if not self.align_marker_to_center():
            self.get_logger().error('Failed to find/align ArUco marker!')
            return
        
        # Phase 2: Calibration
        self.run_calibration()


def main(args=None):
    rclpy.init(args=args)
    node = AlignAndCalibrate()
    
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
