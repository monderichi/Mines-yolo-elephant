#!/usr/bin/env python3
"""
Pick Controller Node for YOLO-MoveIt Integration
Subscribes to YOLO detections and commands robot to move to detected objects.

Camera mount: Fixed position, 90cm left of robot base, 2cm above base.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
import json
import numpy as np
from scipy.spatial.transform import Rotation as R


class PickController(Node):
    def __init__(self):
        super().__init__('pick_controller')
        
        # Parameters
        self.declare_parameter('target_class', 'mouse')
        self.declare_parameter('approach_height', 0.05)  # 5cm above object
        self.declare_parameter('min_confidence', 0.5)
        
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value
        self.approach_height = self.get_parameter('approach_height').get_parameter_value().double_value
        self.min_confidence = self.get_parameter('min_confidence').get_parameter_value().double_value
        
        # Camera transform: camera is 90cm (0.9m) to the LEFT (Y+) of robot base
        # and 2cm (0.02m) above base
        # Camera optical frame: Z forward, X right, Y down
        # Robot base frame: X forward, Y left, Z up
        self.camera_position = np.array([0.0, 0.90, 0.02])  # x, y, z in robot base frame
        
        # Camera rotation: camera looks toward robot (rotated 90° around Z, then tilted)
        # Assuming camera faces the robot workspace
        # This may need adjustment based on actual camera orientation
        self.camera_rotation = R.from_euler('zyx', [np.pi/2, 0, 0])  # 90° yaw to face robot
        
        # Subscribe to YOLO detections
        self.detection_sub = self.create_subscription(
            String,
            '/yolo/detections',
            self.detection_callback,
            10
        )
        
        # MoveIt action client
        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        
        # State
        self.is_moving = False
        self.last_target = None
        
        self.get_logger().info(f'Pick Controller initialized')
        self.get_logger().info(f'Target class: {self.target_class}')
        self.get_logger().info(f'Camera position (robot frame): {self.camera_position}')
    
    def transform_to_robot_frame(self, camera_point):
        """
        Transform point from camera optical frame to robot base frame.
        
        Camera is 90cm LEFT of robot base, looking TOWARD the robot.
        Camera optical frame: Z forward, X right, Y down
        Robot base frame: X forward, Y left, Z up
        
        With camera on left looking right (toward robot):
        - Camera Z (forward) -> Robot -Y (toward robot center)
        - Camera X (right) -> Robot -X (backward from robot's perspective)
        - Camera Y (down) -> Robot -Z (down)
        """
        # Point in camera frame
        cam_x = camera_point['x']
        cam_y = camera_point['y']
        cam_z = camera_point['z']
        
        # Transform to robot frame:
        # Camera Z forward becomes distance toward robot (reducing Y from 0.9)
        # Camera X right becomes robot -X
        # Camera Y down becomes robot -Z
        robot_x = -cam_x
        robot_y = self.camera_position[1] - cam_z  # 0.9m - distance to object
        robot_z = self.camera_position[2] - cam_y  # 0.02m - vertical offset
        
        return np.array([robot_x, robot_y, robot_z])
    
    def detection_callback(self, msg):
        """Process YOLO detection and command robot if target found."""
        if self.is_moving:
            return
            
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        
        # Find target object
        target = None
        for det in detections:
            if det['class'].lower() == self.target_class.lower():
                self.get_logger().info(f'Found {self.target_class}, conf={det["confidence"]:.2f}, has_3d={("position_3d" in det)}')
                if det['confidence'] >= self.min_confidence:
                    if 'position_3d' in det:
                        target = det
                        break
                    else:
                        self.get_logger().warn(f'{self.target_class} detected but no 3D position (depth not available)')
        
        if target is None:
            return
        
        # Check if this is a new target (moved significantly)
        pos_3d = target['position_3d']
        if self.last_target is not None:
            dist = np.sqrt(
                (pos_3d['x'] - self.last_target['x'])**2 +
                (pos_3d['y'] - self.last_target['y'])**2 +
                (pos_3d['z'] - self.last_target['z'])**2
            )
            if dist < 0.05:  # Less than 5cm movement, ignore (increased from 2cm)
                return
        
        self.last_target = pos_3d
        
        # Transform to robot frame
        robot_pos = self.transform_to_robot_frame(pos_3d)
        
        self.get_logger().info(f'>>> MOVING TO {self.target_class} <<<')
        self.get_logger().info(f'Camera frame: ({pos_3d["x"]:.3f}, {pos_3d["y"]:.3f}, {pos_3d["z"]:.3f})')
        self.get_logger().info(f'Robot frame: ({robot_pos[0]:.3f}, {robot_pos[1]:.3f}, {robot_pos[2]:.3f})')
        
        # Move to target (with approach height offset)
        self.move_to_position(robot_pos[0], robot_pos[1], robot_pos[2] + self.approach_height)
    
    def move_to_position(self, x, y, z):
        """Plan and execute move to Cartesian position."""
        if not self._move_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('MoveGroup action server not available')
            return
        
        self.is_moving = True
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        
        # Gripper pointing down (end-effector Z pointing down)
        # Quaternion for Z-down orientation
        goal_pose.pose.orientation.x = 1.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0
        
        # Create motion plan request
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header = goal_pose.header
        position_constraint.link_name = 'link6'  # End-effector link
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        
        # Define constraint region (small sphere around target)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # 1cm radius tolerance
        position_constraint.constraint_region.primitives.append(primitive)
        position_constraint.constraint_region.primitive_poses.append(goal_pose.pose)
        position_constraint.weight = 1.0
        
        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = goal_pose.header
        orientation_constraint.link_name = 'link6'
        orientation_constraint.orientation = goal_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 0.5
        
        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(constraints)
        
        goal_msg.planning_options.plan_only = False  # Plan and execute
        
        self.get_logger().info(f'Planning move to ({x:.3f}, {y:.3f}, {z:.3f})')
        
        # Send goal asynchronously
        send_goal_future = self._move_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.is_moving = False
            return
        
        self.get_logger().info('Goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """Handle execution result."""
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('✓ Move completed successfully!')
        else:
            self.get_logger().warn(f'Move failed with error code: {result.error_code.val}')
        
        self.is_moving = False


def main(args=None):
    rclpy.init(args=args)
    node = PickController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
