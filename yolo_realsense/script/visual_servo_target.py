#!/usr/bin/env python3
"""
Target-Based Visual Servoing for myCobot 320 M5 + RealSense D455

Approach:
1. **Search**: Move robot to a known "home" position where marker should be visible.
2. **Detect**: Find ArUco marker in Camera Frame.
3. **Calculate**:
   - Convert current Marker position to Base Frame using Calibration.
   - Define Target Marker position in Base Frame (centered in camera).
   - Compute delta vector (Target - Current).
   - Apply delta to current End-Effector pose.
4. **Execute**: Send new EE pose to MoveIt2 to solve IK and move.

Usage:
    ros2 run yolo_realsense visual_servo_target.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (MotionPlanRequest, Constraints, 
                              PositionConstraint, OrientationConstraint, JointConstraint)
from shape_msgs.msg import SolidPrimitive
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_ros
from tf2_ros import TransformException
import time


class VisualServoTarget(Node):
    def __init__(self):
        super().__init__('visual_servo_target')
        
        # Parameters
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.03)
        self.declare_parameter('target_distance', 0.5)
        self.declare_parameter('tolerance_xy', 0.01)
        self.declare_parameter('tolerance_z', 0.015)
        
        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.tolerance_xy = self.get_parameter('tolerance_xy').get_parameter_value().double_value
        self.tolerance_z = self.get_parameter('tolerance_z').get_parameter_value().double_value
        
        # Calibration from previous run (T_base_to_camera)
        # Result: X=-0.0444, Y=+1.2601, Z=+0.5974, R=-105.72, P=-12.74, Y=+174.94
        # Note: Previous script output was Camera pos in Base frame.
        r = R.from_euler('xyz', [-105.72, -12.74, 174.94], degrees=True)
        R_matrix = r.as_matrix()
        t_vec = np.array([-0.0444, 1.2601, 0.5974])
        
        # T_camera_to_base (Transform that takes point in Camera and puts it in Base)
        self.T_cam_to_base = np.eye(4)
        self.T_cam_to_base[:3, :3] = R_matrix
        self.T_cam_to_base[:3, 3] = t_vec
        
        # Determine T_base_to_camera (inverse) for later consistency checks if needed
        self.T_base_to_cam = np.linalg.inv(self.T_cam_to_base)
        
        # ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())
        
        # Camera
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_image = None
        
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        
        # MoveIt Actions
        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        self._exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.ee_link = 'link6'
        self.base_link = 'base_link'
        
        self.get_logger().info('Visual Servo Target Node initialized')
        self.get_logger().info(f'  Target distance: {self.target_distance}m')
        
    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received')

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def detect_marker(self):
        """Detect ArUco marker and return position in camera frame."""
        if self.latest_image is None or self.camera_matrix is None:
            return None
        
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        
        if ids is None:
            return None
        
        for i, mid in enumerate(ids.flatten()):
            if mid == self.marker_id:
                marker_corners = corners[i][0]
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
                    return tvec.flatten()
        return None

    def get_current_ee_pose(self):
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

    def home_robot(self):
        """Move robot to a known position where camera can see it."""
        self.get_logger().info('Homing robot to search position...')
        
        # Joint values (approx -160 degrees on J1)
        # Based on calibration script values
        joints = [-2.79, -0.3, 0.3, 0.0, 0.0, 0.0]
        joint_names = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
                       'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
        
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            return False
            
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        constraints = Constraints()
        for name, pos in zip(joint_names, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = True
        
        # Plan
        send_future = self._move_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
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
            
        exec_res_future = exec_handle.get_result_async()
        rclpy.spin_until_future_complete(self, exec_res_future)
        
        return True

    def move_to_pose(self, x, y, z, qx, qy, qz, qw):
        """Use MoveIt to execute a Cartesian move."""
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
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Position constraint (sphere around target)
        pos_constraint = PositionConstraint()
        pos_constraint.header = goal_pose.header
        pos_constraint.link_name = self.ee_link
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # 1cm precision
        pos_constraint.constraint_region.primitives.append(primitive)
        pos_constraint.constraint_region.primitive_poses.append(goal_pose.pose)
        pos_constraint.weight = 1.0
        
        # Orientation (keep same as current)
        orient_constraint = OrientationConstraint()
        orient_constraint.header = goal_pose.header
        orient_constraint.link_name = self.ee_link
        orient_constraint.orientation = goal_pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.2
        orient_constraint.absolute_y_axis_tolerance = 0.2
        orient_constraint.absolute_z_axis_tolerance = 0.2
        orient_constraint.weight = 1.0
        
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = True  # Plan first
        
        self.get_logger().info(f'Planning to ({x:.3f}, {y:.3f}, {z:.3f})...')
        
        # 1. Request Plan
        send_future = self._move_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        
        if result.error_code.val != 1:
            self.get_logger().warn(f'Planning failed: {result.error_code.val}')
            return False
            
        self.get_logger().info('Planning SUCCESS. Executing...')
        
        # 2. Execute Plan
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = result.planned_trajectory
        
        exec_future = self._exec_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, exec_future)
        exec_handle = exec_future.result()
        if not exec_handle.accepted:
            self.get_logger().error('Execution rejected')
            return False
            
        exec_res_future = exec_handle.get_result_async()
        
        # Wait for execution
        start_time = time.time()
        while not exec_res_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.01)
            if time.time() - start_time > 10.0:
                self.get_logger().warn('Execution timed out')
                return False
                
        status = exec_res_future.result().error_code.val
        if status == 1:
            self.get_logger().info('Movement COMPLETE')
            return True
        else:
            self.get_logger().warn(f'Movement failed with code: {status}')
            return False

    def run(self):
        self.get_logger().info('Waiting for camera...')
        while self.camera_matrix is None or self.latest_image is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info('Camera OK.')
        
        # 1. Home Robot
        self.home_robot()
        time.sleep(1.0)
        
        # 2. Loop
        for i in range(10):  # Limited iterations
            self.get_logger().info(f'--- Servo Iteration {i+1} ---')
            
            # Spin to get latest image
            for _ in range(5): rclpy.spin_once(self, timeout_sec=0.1)
            
            # Detect
            marker_tvec = self.detect_marker()
            if marker_tvec is None:
                self.get_logger().warn('Marker not seen!')
                time.sleep(1.0)
                continue
                
            self.get_logger().info(f'Marker (Cam): {marker_tvec}')
            
            # Check convergence
            if (abs(marker_tvec[0]) < self.tolerance_xy and 
                abs(marker_tvec[1]) < self.tolerance_xy and 
                abs(marker_tvec[2] - self.target_distance) < self.tolerance_z):
                self.get_logger().info('ALIGNED!')
                break
                
            # Current EE
            ee_pos, ee_quat = self.get_current_ee_pose()
            if ee_pos is None: continue
            
            # Calculate Target Point
            # P_target_cam = [0, 0, target_dist]
            target_pt_cam = np.array([0.0, 0.0, self.target_distance, 1.0])
            marker_pt_cam = np.append(marker_tvec, 1.0)
            
            # Transform to Base
            # P_target_base = T_cam_to_base * P_target_cam
            target_pt_base = self.T_cam_to_base @ target_pt_cam
            marker_pt_base = self.T_cam_to_base @ marker_pt_cam
            
            # Delta in Base Frame required to move Marker to Target
            # We move EE by same delta
            delta_base = target_pt_base[:3] - marker_pt_base[:3]
            
            # Scale delta (Gain) to avoid overshoot
            gain = 0.5
            delta_base *= gain
            
            # Clamp delta
            max_d = 0.05
            delta_base = np.clip(delta_base, -max_d, max_d)
            
            new_ee_pos = ee_pos + delta_base
            new_ee_pos[2] = max(new_ee_pos[2], 0.05) # Floor safety
            
            self.get_logger().info(f'Delta (Base): {delta_base}')
            
            # Move
            self.move_to_pose(
                new_ee_pos[0], new_ee_pos[1], new_ee_pos[2],
                ee_quat[0], ee_quat[1], ee_quat[2], ee_quat[3]
            )
            
            time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoTarget()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
