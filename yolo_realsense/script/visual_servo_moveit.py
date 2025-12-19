#!/usr/bin/env python3
"""
Visual Servoing with MoveGroup (no MoveIt Servo required)

Uses MoveGroup action client to make incremental Cartesian moves
based on ArUco marker detection. Works with position-only controllers.

Usage:
    ros2 run yolo_realsense visual_servo_moveit.py
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
import tf2_ros
from tf2_ros import TransformException
import time


class VisualServoMoveIt(Node):
    def __init__(self):
        super().__init__('visual_servo_moveit')
        
        # Parameters
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.03)
        self.declare_parameter('target_distance', 0.5)
        self.declare_parameter('step_size', 0.02)  # 2cm steps
        self.declare_parameter('deadband', 0.015)  # 15mm deadband
        
        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.step_size = self.get_parameter('step_size').get_parameter_value().double_value
        self.deadband = self.get_parameter('deadband').get_parameter_value().double_value
        
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
        
        # MoveIt
        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        self._exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.ee_link = 'link6'
        self.base_link = 'base_link'
        
        self.is_moving = False
        
        self.get_logger().info('Visual Servo MoveIt Node initialized')
        self.get_logger().info(f'  Target distance: {self.target_distance}m, step: {self.step_size}m')
    
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
    
    def detect_aruco(self):
        """Return tvec or None."""
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
        """Get EE pose from TF2."""
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
        except TransformException:
            return None, None
    
    def move_cartesian(self, x, y, z, qx, qy, qz, qw):
        """Small Cartesian move using MoveGroup."""
        if not self._move_client.wait_for_server(timeout_sec=2.0):
            return False
        if not self._exec_client.wait_for_server(timeout_sec=2.0):
            return False
        
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
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 2.0
        
        pos_constraint = PositionConstraint()
        pos_constraint.header = goal_pose.header
        pos_constraint.link_name = self.ee_link
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.02]
        pos_constraint.constraint_region.primitives.append(primitive)
        pos_constraint.constraint_region.primitive_poses.append(goal_pose.pose)
        pos_constraint.weight = 1.0
        
        orient_constraint = OrientationConstraint()
        orient_constraint.header = goal_pose.header
        orient_constraint.link_name = self.ee_link
        orient_constraint.orientation = goal_pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.3
        orient_constraint.absolute_y_axis_tolerance = 0.3
        orient_constraint.absolute_z_axis_tolerance = 0.3
        orient_constraint.weight = 0.5
        
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
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
        
        exec_result_future = exec_handle.get_result_async()
        start = time.time()
        while not exec_result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > 10:
                break
        
        return True
    
    def run(self):
        """Main servo loop."""
        self.get_logger().info('Waiting for camera...')
        while self.camera_matrix is None or self.latest_image is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info('Camera ready.')
        
        time.sleep(2.0)  # Wait for TF
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('Starting visual servoing loop...')
        self.get_logger().info('Target: center marker in camera, distance = {:.2f}m'.format(self.target_distance))
        
        while rclpy.ok():
            # Update camera
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.05)
            
            # Detect marker
            tvec = self.detect_aruco()
            if tvec is None:
                self.get_logger().warn('Marker not visible, waiting...')
                time.sleep(0.5)
                continue
            
            marker_x, marker_y, marker_z = tvec
            
            # Compute errors
            error_x = marker_x  # Should be 0 (centered horizontally)
            error_y = marker_y  # Should be 0 (centered vertically)
            error_z = marker_z - self.target_distance  # Should be at target distance
            
            self.get_logger().info(
                f'Marker: ({marker_x*1000:.1f}, {marker_y*1000:.1f}, {marker_z*1000:.1f})mm, '
                f'err: ({error_x*1000:.1f}, {error_y*1000:.1f}, {error_z*1000:.1f})mm'
            )
            
            # Check if converged
            if abs(error_x) < self.deadband and abs(error_y) < self.deadband and abs(error_z) < self.deadband:
                self.get_logger().info('✓ ALIGNED! Errors within deadband.')
                time.sleep(1.0)
                continue
            
            # Get current EE pose
            pos, quat = self.get_current_ee_pose()
            if pos is None:
                self.get_logger().warn('Cannot get EE pose from TF')
                time.sleep(0.5)
                continue
            
            # Compute Cartesian adjustment
            # Simple proportional control with step limiting
            gain = 0.5
            
            # Camera frame to robot frame mapping (approximate)
            # Assuming camera looks at robot from a fixed position
            delta_x = -gain * error_z  # Move forward/back
            delta_y = -gain * error_x  # Move left/right
            delta_z = -gain * error_y  # Move up/down
            
            # Limit step size
            delta_x = np.clip(delta_x, -self.step_size, self.step_size)
            delta_y = np.clip(delta_y, -self.step_size, self.step_size)
            delta_z = np.clip(delta_z, -self.step_size, self.step_size)
            
            new_x = pos[0] + delta_x
            new_y = pos[1] + delta_y
            new_z = max(pos[2] + delta_z, 0.05)  # Keep above ground
            
            self.get_logger().info(f'  Moving by ({delta_x*1000:.1f}, {delta_y*1000:.1f}, {delta_z*1000:.1f})mm')
            
            if not self.move_cartesian(new_x, new_y, new_z, quat[0], quat[1], quat[2], quat[3]):
                self.get_logger().warn('Move failed')
            
            time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoMoveIt()
    
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
