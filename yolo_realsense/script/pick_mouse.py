#!/usr/bin/env python3
"""
Pick Mouse: Detects a mouse via YOLO and moves the robot arm to it.
Uses TF to transform camera coordinates to robot base frame.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import json


class PickMouse(Node):
    def __init__(self):
        super().__init__('pick_mouse')

        self.declare_parameter('target_class', 'mouse')
        self.target_class = self.get_parameter('target_class').value

        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveIt Client
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

        # YOLO Sub
        self.subscription = self.create_subscription(
            String,
            '/yolo/detections',
            self.detection_callback,
            10)

        self.is_moving = False
        self.get_logger().info('Pick Mouse Node Ready. Waiting for detections...')

    def detection_callback(self, msg):
        if self.is_moving:
            return

        try:
            detections = json.loads(msg.data)
        except:
            return

        for det in detections:
            if det['class'] == self.target_class:
                if 'position_3d' in det:
                    pos = det['position_3d']
                    self.get_logger().info(f"Mouse detected at Camera Frame: {pos}")
                    self.initiate_pick(pos)
                    return

    def initiate_pick(self, pos_cam):
        self.is_moving = True

        # Create PointStamped in Camera Frame
        p_cam = PointStamped()
        p_cam.header.frame_id = "camera_color_optical_frame"
        p_cam.header.stamp = self.get_clock().now().to_msg()
        p_cam.point.x = float(pos_cam['x'])
        p_cam.point.y = float(pos_cam['y'])
        p_cam.point.z = float(pos_cam['z'])

        # Transform to base_link
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", "camera_color_optical_frame",
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            p_base = do_transform_point(p_cam, transform)
        except Exception as e:
            self.get_logger().error(f"TF Error: {e}")
            self.is_moving = False
            return

        target_x = p_base.point.x
        target_y = p_base.point.y
        target_z = max(p_base.point.z, 0.05)  # At least 5cm above ground

        self.get_logger().info(f"Target in Base Frame: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")

        # Send MoveIt goal (approach height = target_z + 2cm to touch)
        self.send_moveit_goal(target_x, target_y, target_z + 0.02)

    def send_moveit_goal(self, x, y, z):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('MoveGroup action server not available!')
            self.is_moving = False
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Safety: Move slowly
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1

        # Position Constraint
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "link6"
        pc.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.02]))

        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import OrientationConstraint
        
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        
        # Orientation: End effector pointing DOWN (Z-axis pointing down)
        # Quaternion for 180° rotation around X axis (pointing down)
        target_pose.orientation.x = 1.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.0
        
        pc.constraint_region.primitive_poses.append(target_pose)
        pc.weight = 1.0

        # Orientation Constraint
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "link6"
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.4  # ~23 degrees tolerance
        oc.absolute_y_axis_tolerance = 0.4
        oc.absolute_z_axis_tolerance = 3.14  # Free rotation around Z
        oc.weight = 1.0

        goal_msg.request.goal_constraints.append(
            Constraints(position_constraints=[pc], orientation_constraints=[oc]))
        goal_msg.planning_options.plan_only = False  # Plan AND Execute

        self.get_logger().info(f"Sending MoveIt goal to ({x:.3f}, {y:.3f}, {z:.3f}) - pointing DOWN")

        # Send goal asynchronously (non-blocking)
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by MoveGroup!')
            self.is_moving = False
            return

        self.get_logger().info('Goal accepted. Executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('SUCCESS! Robot reached target position.')
        else:
            self.get_logger().warn(f'Motion failed. Error code: {result.error_code.val}')

        self.is_moving = False
        self.get_logger().info('Ready for next detection.')


def main(args=None):
    rclpy.init(args=args)
    node = PickMouse()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
