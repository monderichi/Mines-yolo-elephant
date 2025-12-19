#!/usr/bin/env python3
"""
Cartesian IK Calibration Script for myCobot 320 M5 + RealSense D455

Uses MoveIt2's built-in Inverse Kinematics to:
1. Detect ArUco marker and determine where to move
2. Command end-effector to align with camera center (XY) using Cartesian moves
3. Collect samples at different distances
4. Solve hand-eye calibration

Usage:
    ros2 run yolo_realsense cartesian_calibrate.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (MotionPlanRequest, Constraints, 
                              PositionConstraint, OrientationConstraint)
from shape_msgs.msg import SolidPrimitive
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import json
from datetime import datetime
import tf2_ros
from tf2_ros import TransformException


class CartesianCalibrator(Node):
    def __init__(self):
        super().__init__('cartesian_calibrator')
        
        # ------ Parameters ------
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.03)  # 30mm marker
        self.declare_parameter('num_samples', 12)
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        
        aruco_dict_name = self.get_parameter('aruco_dict').get_parameter_value().string_value
        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.num_samples = self.get_parameter('num_samples').get_parameter_value().integer_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        
        # ArUco
        aruco_dicts = {'DICT_4X4_50': cv2.aruco.DICT_4X4_50}
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dicts.get(aruco_dict_name, cv2.aruco.DICT_4X4_50))
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Camera
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_image = None
        self.image_width = 0
        self.image_height = 0
        
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        
        # MoveIt
        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        self._exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        # TF2 for getting actual end-effector pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # End-effector link name
        self.ee_link = 'link6'
        self.base_link = 'base_link'
        
        # Calibration data
        self.R_gripper2base_list = []
        self.t_gripper2base_list = []
        self.R_target2cam_list = []
        self.t_target2cam_list = []
        
        # Current end-effector pose (will be read from TF)
        self.current_ee_pose = None
        
        self.get_logger().info('Cartesian IK Calibrator initialized')
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
        """Return (center_x, center_y, rvec, tvec) or None."""
        if self.latest_image is None or self.camera_matrix is None:
            return None
        
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        
        if ids is None:
            return None
        
        for i, mid in enumerate(ids.flatten()):
            if mid == self.marker_id:
                marker_corners = corners[i][0]
                center_x = np.mean(marker_corners[:, 0])
                center_y = np.mean(marker_corners[:, 1])
                
                half_size = self.marker_size / 2.0
                obj_points = np.array([
                    [-half_size, half_size, 0],
                    [half_size, half_size, 0],
                    [half_size, -half_size, 0],
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
    
    def get_current_ee_pose(self):
        """Get current end-effector pose from TF2."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_link, self.ee_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            pos = np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z
            ])
            
            quat = np.array([
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w
            ])
            
            return pos, quat
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None, None
    
    def move_to_cartesian(self, x, y, z, qx=1.0, qy=0.0, qz=0.0, qw=0.0, wait_time=3.0):
        """Move end-effector to Cartesian position using MoveIt2 IK."""
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup not available')
            return False
        if not self._exec_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('ExecuteTrajectory not available')
            return False
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.base_link
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        
        # Create motion plan request
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 10.0
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = goal_pose.header
        pos_constraint.link_name = self.ee_link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
        # Constraint region (sphere around target)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.02]  # 2cm tolerance
        pos_constraint.constraint_region.primitives.append(primitive)
        pos_constraint.constraint_region.primitive_poses.append(goal_pose.pose)
        pos_constraint.weight = 1.0
        
        # Orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header = goal_pose.header
        orient_constraint.link_name = self.ee_link
        orient_constraint.orientation = goal_pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.2
        orient_constraint.absolute_y_axis_tolerance = 0.2
        orient_constraint.absolute_z_axis_tolerance = 0.2
        orient_constraint.weight = 0.5
        
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = True
        
        self.get_logger().info(f'Planning to ({x:.3f}, {y:.3f}, {z:.3f})...')
        
        # Plan
        send_goal_future = self._move_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Planning goal rejected')
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.error_code.val != 1:
            self.get_logger().warn(f'Planning failed: {result.error_code.val}')
            return False
        
        self.get_logger().info('Planning OK, executing...')
        
        # Execute
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
        
        time.sleep(wait_time)
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
        
        return True
    
    def align_with_aruco(self):
        """Find ArUco marker using joint-space moves (which works better for initial positioning)."""
        self.get_logger().info('='*60)
        self.get_logger().info('PHASE 1: Finding ArUco marker')
        self.get_logger().info('='*60)
        
        # Use joint-space positioning first (camera at -160 degrees = -2.79 rad)
        # This is more reliable for initial positioning
        from moveit_msgs.msg import JointConstraint
        
        joint_names = [
            'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
            'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'
        ]
        
        base_j1 = -2.79  # -160 degrees toward camera
        
        # Poses to try (joint space)
        search_poses = [
            [base_j1, -0.3, 0.3, 0.0, 0.0, 0.0],
            [base_j1, -0.2, 0.2, 0.0, 0.0, 0.0],
            [base_j1, -0.4, 0.4, 0.0, 0.0, 0.0],
            [base_j1 + 0.15, -0.3, 0.3, 0.0, 0.0, 0.0],
            [base_j1 - 0.15, -0.3, 0.3, 0.0, 0.0, 0.0],
        ]
        
        self.get_logger().info('Searching for marker using joint-space moves...')
        
        for i, joints in enumerate(search_poses):
            self.get_logger().info(f'Search pose {i+1}/{len(search_poses)}')
            
            if not self.move_to_joints(joints, joint_names):
                continue
            
            time.sleep(2.5)
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.05)
            
            detection = self.detect_aruco()
            if detection is not None:
                center_x, center_y, rvec, tvec = detection
                self.get_logger().info(f'✓ Marker found! tvec={tvec}, pixel=({center_x:.0f}, {center_y:.0f})')
                return True
        
        self.get_logger().error('Could not find ArUco marker')
        return False
    
    def move_to_joints(self, target_joints, joint_names, wait_time=3.0):
        """Move to joint configuration using MoveIt."""
        from moveit_msgs.msg import JointConstraint
        
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
        for name, position in zip(joint_names, target_joints):
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
        
        time.sleep(wait_time)
        return True
    
    def collect_samples(self):
        """Collect samples at different positions."""
        self.get_logger().info('='*60)
        self.get_logger().info('PHASE 2: Collecting calibration samples')
        self.get_logger().info('='*60)
        
        pos, quat = self.get_current_ee_pose()
        if pos is None:
            self.get_logger().error('Cannot get current pose')
            return 0
        
        base_x, base_y, base_z = pos
        
        # Variations in Cartesian space
        variations = [
            [0.0, 0.0, 0.0],
            [0.02, 0.0, 0.0],
            [-0.02, 0.0, 0.0],
            [0.0, 0.02, 0.0],
            [0.0, -0.02, 0.0],
            [0.0, 0.0, 0.02],
            [0.0, 0.0, -0.02],
            [0.03, 0.03, 0.0],
            [-0.03, 0.03, 0.0],
            [0.0, 0.0, 0.04],
            [0.0, 0.0, -0.04],
            [0.04, 0.0, 0.02],
        ]
        
        samples = 0
        
        for i, (dx, dy, dz) in enumerate(variations[:self.num_samples]):
            self.get_logger().info(f'\n--- Sample {i+1}/{min(len(variations), self.num_samples)} ---')
            
            target_x = base_x + dx
            target_y = base_y + dy
            target_z = max(base_z + dz, 0.05)
            
            self.get_logger().info(f'Moving to ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})')
            if not self.move_to_cartesian(target_x, target_y, target_z, quat[0], quat[1], quat[2], quat[3], wait_time=2.5):
                self.get_logger().warn('Move failed')
                continue
            
            detection = self.detect_aruco()
            if detection is None:
                self.get_logger().warn('ArUco not detected')
                continue
            
            center_x, center_y, rvec, tvec = detection
            self.get_logger().info(f'✓ Detected: tvec=({tvec[0]*1000:.1f}, {tvec[1]*1000:.1f}, {tvec[2]*1000:.1f})mm')
            
            # Get actual EE pose from TF
            ee_pos, ee_quat = self.get_current_ee_pose()
            if ee_pos is None:
                self.get_logger().warn('Cannot get EE pose')
                continue
            
            # Convert EE pose to rotation matrix and translation
            r_ee = R.from_quat(ee_quat)
            R_base_ee = r_ee.as_matrix()
            t_base_ee = ee_pos
            
            # T_gripper2base = inv(T_base_gripper)
            R_ee2base = R_base_ee.T
            t_ee2base = -R_base_ee.T @ t_base_ee
            
            # Marker pose in camera frame
            R_marker2cam, _ = cv2.Rodrigues(rvec)
            t_marker2cam = tvec
            
            # Store
            self.R_gripper2base_list.append(R_ee2base)
            self.t_gripper2base_list.append(t_ee2base.reshape(3, 1))
            self.R_target2cam_list.append(R_marker2cam)
            self.t_target2cam_list.append(t_marker2cam.reshape(3, 1))
            
            samples += 1
            self.get_logger().info(f'Sample {samples} collected')
        
        return samples
    
    def solve_calibration(self, samples):
        """Solve and print calibration."""
        if samples < 4:
            self.get_logger().error(f'Only {samples} samples. Need ≥4.')
            return
        
        self.get_logger().info(f'\nSolving with {samples} samples...')
        
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
        self.get_logger().info(f'\nCamera position (m): X={pos[0]:+.4f}, Y={pos[1]:+.4f}, Z={pos[2]:+.4f}')
        self.get_logger().info(f'Euler (deg): R={euler[0]:+.2f}, P={euler[1]:+.2f}, Y={euler[2]:+.2f}')
        self.get_logger().info(f'\n*** DISTANCE: {distance:.4f} m ({distance*1000:.2f} mm) ***')
        
        result = {
            'timestamp': datetime.now().isoformat(),
            'samples': samples,
            'T_camera_to_base': T_cam2base.tolist(),
            'position': pos.tolist(),
            'distance_m': float(distance),
        }
        
        filename = f'cartesian_calibration_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
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
    
    def run(self):
        """Main."""
        self.get_logger().info('Waiting for camera...')
        start = time.time()
        while self.camera_matrix is None or self.latest_image is None:
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - start > 30:
                self.get_logger().error('Camera not available!')
                return
        self.get_logger().info('Camera ready.')
        
        # Wait for TF
        time.sleep(2.0)
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Phase 1
        if not self.align_with_aruco():
            self.get_logger().error('Alignment failed!')
            return
        
        # Phase 2
        samples = self.collect_samples()
        
        # Phase 3
        self.solve_calibration(samples)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianCalibrator()
    
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
