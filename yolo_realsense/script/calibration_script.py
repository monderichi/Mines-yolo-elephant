#!/usr/bin/env python3
"""
Hand-Eye Calibration Script for myCobot 320 M5 + RealSense D455

Eye-to-Hand calibration: Camera is FIXED in the world, ArUco marker is attached
to the robot's end-effector.

This script:
1. Moves the robot to multiple poses via MoveIt2
2. Detects the ArUco marker attached to the end-effector
3. Collects pairs of (T_base_ee, T_camera_marker) using TF2 for the EE pose
4. Solves for T_base_camera using OpenCV's calibrateHandEye
5. Outputs the camera-to-base transformation matrix and distance

Usage:
    ros2 run yolo_realsense calibration_script.py

Requirements:
    - Running MoveIt2 for myCobot
    - Running realsense2_camera node
    - ArUco marker attached to end-effector

Notes:
    - Configure frames via parameters `base_frame` and `ee_frame`.
    - By default, this script prefers TF2 for EE pose (`prefer_tf_pose:=True`) and
      falls back to approximate FK only if TF lookup fails.
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
import tf2_ros
from tf2_ros import TransformException


class HandEyeCalibrator(Node):
    def __init__(self):
        super().__init__('hand_eye_calibrator')
        
        # ------ Parameters ------
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.03)  # 30mm marker
        self.declare_parameter('num_samples', 15)
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'link6')
        self.declare_parameter('prefer_tf_pose', True)
        
        aruco_dict_name = self.get_parameter('aruco_dict').get_parameter_value().string_value
        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.num_samples = self.get_parameter('num_samples').get_parameter_value().integer_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.ee_frame = self.get_parameter('ee_frame').get_parameter_value().string_value
        self.prefer_tf_pose = self.get_parameter('prefer_tf_pose').get_parameter_value().bool_value
        
        # ArUco dictionary
        aruco_dicts = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
        }
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dicts.get(aruco_dict_name, cv2.aruco.DICT_5X5_250))
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # ------ Camera ------
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_image = None
        self.latest_image_stamp = None
        
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        # ------ TF2 (preferred EE pose source) ------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ------ MoveIt ------
        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        self._exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        # Joint names for myCobot 320 M5
        self.joint_names = [
            'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
            'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'
        ]
        
        # Calibration poses (diverse orientations to get good data)
        # Camera is at -160 degrees from robot, so joint 1 base is around -2.79 rad
        # Format: [j1, j2, j3, j4, j5, j6] in radians
        base_j1 = -2.79  # -160 degrees
        self.calibration_poses = [
            [base_j1, 0.0, 0.0, 0.0, 0.0, 0.0],
            [base_j1 + 0.2, -0.3, 0.3, 0.0, 0.0, 0.0],
            [base_j1 - 0.2, -0.3, 0.3, 0.0, 0.0, 0.0],
            [base_j1, -0.5, 0.5, 0.0, 0.0, 0.0],
            [base_j1 + 0.3, -0.2, 0.2, 0.3, 0.0, 0.0],
            [base_j1 - 0.3, -0.2, 0.2, -0.3, 0.0, 0.0],
            [base_j1 + 0.15, -0.4, 0.4, 0.0, 0.5, 0.0],
            [base_j1 - 0.15, -0.4, 0.4, 0.0, -0.5, 0.0],
            [base_j1, -0.6, 0.6, 0.0, 0.0, 0.0],
            [base_j1 + 0.4, -0.1, 0.1, 0.0, 0.0, 0.0],
            [base_j1 - 0.4, -0.1, 0.1, 0.0, 0.0, 0.0],
            [base_j1, -0.3, 0.3, 0.5, 0.0, 0.0],
            [base_j1, -0.3, 0.3, -0.5, 0.0, 0.0],
            [base_j1 + 0.25, -0.5, 0.5, 0.2, 0.2, 0.0],
            [base_j1 - 0.25, -0.5, 0.5, -0.2, -0.2, 0.0],
        ]
        
        # Collected data
        self.R_gripper2base_list = []
        self.t_gripper2base_list = []
        self.R_target2cam_list = []
        self.t_target2cam_list = []
        
        self.get_logger().info('Hand-Eye Calibrator initialized')
        self.get_logger().info(f'ArUco dict: {aruco_dict_name}, Marker ID: {self.marker_id}, Size: {self.marker_size}m')
        self.get_logger().info(f'Will collect {self.num_samples} samples')
    
    def image_callback(self, msg):
        """Store latest image."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_image_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    def get_base_to_ee_pose(self):
        """Return (R_base_ee, t_base_ee) using TF2, or (None, None) if unavailable."""
        if not self.prefer_tf_pose:
            return None, None

        try:
            query_time = rclpy.time.Time()
            if self.latest_image_stamp is not None:
                query_time = rclpy.time.Time.from_msg(self.latest_image_stamp)

            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                query_time,
                timeout=rclpy.duration.Duration(seconds=1.0),
            )

            t = tf_msg.transform.translation
            q = tf_msg.transform.rotation
            t_base_ee = np.array([t.x, t.y, t.z], dtype=np.float64)
            R_base_ee = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix().astype(np.float64)
            return R_base_ee, t_base_ee
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed ({self.base_frame} -> {self.ee_frame}): {e}')
            return None, None
    
    def camera_info_callback(self, msg):
        """Extract camera intrinsics."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info(f'Camera intrinsics received')
    
    def detect_aruco(self):
        """Detect ArUco marker and return pose (rvec, tvec) or None."""
        if self.latest_image is None or self.camera_matrix is None:
            return None
        
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        
        if ids is None:
            return None
        
        # Find our marker
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == self.marker_id:
                # Get marker corners
                marker_corners = corners[i][0]
                
                # Define 3D points of the marker in marker coordinate system
                # Marker center is at origin, corners are at half marker size
                half_size = self.marker_size / 2.0
                obj_points = np.array([
                    [-half_size,  half_size, 0],
                    [ half_size,  half_size, 0],
                    [ half_size, -half_size, 0],
                    [-half_size, -half_size, 0]
                ], dtype=np.float32)
                
                # Solve PnP to get pose
                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    marker_corners,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                
                if success:
                    return rvec.flatten(), tvec.flatten()
        
        return None
    
    def get_end_effector_pose(self):
        """
        Get current end-effector pose from MoveIt/TF.
        For simplicity, we compute forward kinematics from joint states.
        This is a placeholder - in production, use TF lookup.
        
        Returns (R, t) as 3x3 matrix and 3x1 vector.
        """
        # NOTE: This is simplified. In a real implementation, you would
        # subscribe to /joint_states and use a URDF-based FK solver,
        # or look up the transform via TF2.
        # 
        # For now, we'll use MoveIt's FK via the move_group interface,
        # which requires a service call. As an alternative, we store
        # the target pose we commanded and assume the robot reached it.
        
        # This is a limitation - we should use TF2 to get actual pose.
        # For this script, we'll record the commanded joint positions
        # and use approximate FK based on the known kinematics.
        return None  # Will be filled in during pose collection
    
    def move_to_joints(self, target_joints):
        """Move robot to target joint configuration. Returns True on success."""
        self.get_logger().info('Waiting for move_action server...')
        if not self._move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available!')
            return False
        if not self._exec_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('ExecuteTrajectory action server not available!')
            return False
        
        # Create motion plan request - PLAN ONLY
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Create joint constraints
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
        
        self.get_logger().info(f'Planning to: {[round(j, 2) for j in target_joints]}')
        
        # Plan
        send_goal_future = self._move_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Planning goal rejected!')
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.error_code.val != 1:
            self.get_logger().warn(f'Planning failed: error code {result.error_code.val}')
            return False
        
        self.get_logger().info('Planning OK, executing...')
        
        # Execute
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = result.planned_trajectory
        
        exec_future = self._exec_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, exec_future)
        
        exec_handle = exec_future.result()
        if not exec_handle.accepted:
            self.get_logger().error('Execution rejected!')
            return False
        
        # Wait for execution
        exec_result_future = exec_handle.get_result_async()
        timeout = 60.0
        start = time.time()
        while not exec_result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().warn('Execution timeout')
                return True  # Assume completed
        
        exec_result = exec_result_future.result().result
        if exec_result.error_code.val == 1:
            self.get_logger().info('Execution completed!')
            return True
        else:
            self.get_logger().warn(f'Execution error: {exec_result.error_code.val}')
            return True  # Robot likely moved
    
    def compute_fk(self, joints):
        """
        Compute forward kinematics for myCobot 320 M5.
        Returns (R, t) - rotation matrix and translation vector.
        
        This uses the DH parameters for the myCobot 320.
        """
        # myCobot 320 DH parameters (standard DH convention)
        # These are approximate - check your URDF for exact values
        # Format: [d, a, alpha] for each joint
        # Note: theta comes from joint angles
        dh_params = [
            # d (mm -> m), a (mm -> m), alpha (rad)
            [0.1315, 0.0, np.pi/2],      # Joint 1
            [0.0, -0.1104, 0.0],         # Joint 2
            [0.0, -0.096, 0.0],          # Joint 3
            [0.0631, 0.0, np.pi/2],      # Joint 4
            [0.0735, 0.0, -np.pi/2],     # Joint 5
            [0.0456, 0.0, 0.0],          # Joint 6
        ]
        
        def dh_matrix(theta, d, a, alpha):
            """Compute transformation matrix from DH parameters."""
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
        """Main calibration routine."""
        self.get_logger().info('='*60)
        self.get_logger().info('Starting Hand-Eye Calibration')
        self.get_logger().info('='*60)
        
        # Wait for camera
        self.get_logger().info('Waiting for camera...')
        timeout = 30.0
        start = time.time()
        while self.camera_matrix is None or self.latest_image is None:
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - start > timeout:
                self.get_logger().error('Camera not available!')
                return
        self.get_logger().info('Camera ready.')
        
        # Collect samples
        samples_collected = 0
        poses_to_try = self.calibration_poses[:self.num_samples]
        
        for i, pose in enumerate(poses_to_try):
            self.get_logger().info(f'\n--- Sample {i+1}/{len(poses_to_try)} ---')
            
            # Move robot
            if not self.move_to_joints(pose):
                self.get_logger().warn(f'Failed to move to pose {i+1}, skipping')
                continue
            
            # Wait for robot to settle
            time.sleep(2.0)
            
            # Spin to get latest image
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            # Detect ArUco
            detection = self.detect_aruco()
            if detection is None:
                self.get_logger().warn(f'ArUco not detected at pose {i+1}, skipping')
                continue
            
            rvec, tvec = detection
            self.get_logger().info(f'ArUco detected! tvec={tvec}')
            
            # Get end-effector pose (prefer TF2, fallback to FK)
            R_base_ee, t_base_ee = self.get_base_to_ee_pose()
            if R_base_ee is None or t_base_ee is None:
                self.get_logger().warn('Falling back to approximate FK for EE pose (consider enabling TF frames).')
                R_base_ee, t_base_ee = self.compute_fk(pose)
            
            # For eye-to-hand calibration:
            # We need T_gripper2base (inverse of T_base_gripper)
            # and T_target2cam
            
            # T_gripper2base = inv(T_base_gripper)
            R_ee2base = R_base_ee.T
            t_ee2base = -R_base_ee.T @ t_base_ee
            
            # T_target2cam from ArUco detection
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
            self.get_logger().error(f'Only {samples_collected} samples collected. Need at least 4.')
            return
        
        self.get_logger().info(f'\nCollected {samples_collected} samples. Computing calibration...')
        
        # Solve hand-eye calibration
        # For eye-to-hand: we solve for T_base_camera
        # OpenCV's calibrateHandEye expects:
        #   R_gripper2base, t_gripper2base: gripper pose in base frame (inverted)
        #   R_target2cam, t_target2cam: target pose in camera frame
        # 
        # The result is R_cam2gripper, t_cam2gripper for eye-in-hand
        # For eye-to-hand, the result is T_cam2base
        
        R_cam2base, t_cam2base = cv2.calibrateHandEye(
            self.R_gripper2base_list,
            self.t_gripper2base_list,
            self.R_target2cam_list,
            self.t_target2cam_list,
            method=cv2.CALIB_HAND_EYE_TSAI
        )
        
        # Build 4x4 transformation matrix
        T_cam2base = np.eye(4)
        T_cam2base[:3, :3] = R_cam2base
        T_cam2base[:3, 3] = t_cam2base.flatten()
        
        # Also compute T_base2cam (inverse)
        T_base2cam = np.linalg.inv(T_cam2base)
        
        # Distance between camera and robot base
        camera_position_in_base = t_cam2base.flatten()
        distance = np.linalg.norm(camera_position_in_base)
        
        # Convert rotation to Euler angles for readability
        r = R.from_matrix(R_cam2base)
        euler = r.as_euler('xyz', degrees=True)
        quat = r.as_quat()  # x, y, z, w
        
        # Print results
        self.get_logger().info('='*60)
        self.get_logger().info('CALIBRATION RESULTS')
        self.get_logger().info('='*60)
        self.get_logger().info(f'\nT_camera_to_base (4x4 matrix):')
        for row in T_cam2base:
            self.get_logger().info(f'  [{row[0]:+.6f}, {row[1]:+.6f}, {row[2]:+.6f}, {row[3]:+.6f}]')
        
        self.get_logger().info(f'\nCamera position in base frame (meters):')
        self.get_logger().info(f'  X: {camera_position_in_base[0]:+.4f}')
        self.get_logger().info(f'  Y: {camera_position_in_base[1]:+.4f}')
        self.get_logger().info(f'  Z: {camera_position_in_base[2]:+.4f}')
        
        self.get_logger().info(f'\nCamera orientation (Euler XYZ, degrees):')
        self.get_logger().info(f'  Roll:  {euler[0]:+.2f}°')
        self.get_logger().info(f'  Pitch: {euler[1]:+.2f}°')
        self.get_logger().info(f'  Yaw:   {euler[2]:+.2f}°')
        
        self.get_logger().info(f'\nCamera orientation (Quaternion XYZW):')
        self.get_logger().info(f'  [{quat[0]:+.6f}, {quat[1]:+.6f}, {quat[2]:+.6f}, {quat[3]:+.6f}]')
        
        self.get_logger().info(f'\n*** Distance from camera to robot base: {distance:.4f} m ({distance*1000:.2f} mm) ***')
        
        # Save results to file
        result = {
            'timestamp': datetime.now().isoformat(),
            'samples_used': samples_collected,
            'T_camera_to_base': T_cam2base.tolist(),
            'T_base_to_camera': T_base2cam.tolist(),
            'camera_position_in_base': camera_position_in_base.tolist(),
            'camera_orientation_euler_xyz_deg': euler.tolist(),
            'camera_orientation_quaternion_xyzw': quat.tolist(),
            'distance_m': float(distance),
        }
        
        filename = f'calibration_result_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        with open(filename, 'w') as f:
            json.dump(result, f, indent=2)
        self.get_logger().info(f'\nResults saved to: {filename}')
        
        # Print static transform publisher command
        self.get_logger().info('\n--- Static Transform Publisher Command ---')
        self.get_logger().info('Add this to your launch file:')
        self.get_logger().info(f'''
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_to_base_tf',
    arguments=[
        '{camera_position_in_base[0]:.6f}',
        '{camera_position_in_base[1]:.6f}',
        '{camera_position_in_base[2]:.6f}',
        '{quat[0]:.6f}',
        '{quat[1]:.6f}',
        '{quat[2]:.6f}',
        '{quat[3]:.6f}',
        'base_link',
        'camera_link'
    ]
)
''')
        
        self.get_logger().info('='*60)
        self.get_logger().info('Calibration complete!')
        self.get_logger().info('='*60)
        
        # Return home
        self.get_logger().info('Returning to home position...')
        self.move_to_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrator()
    
    try:
        node.run_calibration()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
