#!/usr/bin/env python3
"""
Test MoveIt2 motion planning for myCobot 320 M5

This script tests the MoveIt2 setup by:
1. Connecting to the move_group action server
2. Planning a simple motion
3. (Optionally) executing the motion on the simulated robot

Run this after launching: ros2 launch mycobot_320_moveit2 demo.launch.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    RobotState
)
from sensor_msgs.msg import JointState
import math


class MoveIt2Tester(Node):
    def __init__(self):
        super().__init__('moveit2_tester')
        
        # Action client for move_group
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Current joint state
        self.current_joint_state = None
        self._joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self._joint_state_callback,
            10
        )
        
        self.get_logger().info('MoveIt2 Tester initialized')
        
    def _joint_state_callback(self, msg):
        self.current_joint_state = msg
        
    def wait_for_server(self, timeout_sec=10.0):
        """Wait for the move_group action server"""
        self.get_logger().info('Waiting for move_group action server...')
        if self._action_client.wait_for_server(timeout_sec):
            self.get_logger().info('Connected to move_group!')
            return True
        else:
            self.get_logger().error('Could not connect to move_group action server')
            return False
            
    def plan_to_joint_goal(self, joint_values: list, execute: bool = False):
        """
        Plan (and optionally execute) a motion to specified joint values.
        
        Args:
            joint_values: List of 6 joint values in radians
            execute: Whether to execute the planned motion
        """
        self.get_logger().info(f'Planning motion to joint values: {joint_values}')
        
        # Create motion plan request
        goal = MoveGroup.Goal()
        
        # Set request parameters
        request = MotionPlanRequest()
        request.group_name = 'arm_group'
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5
        
        # Set the start state from current
        if self.current_joint_state:
            request.start_state.joint_state = self.current_joint_state
        
        # Joint names for myCobot 320
        joint_names = [
            'joint2_to_joint1',
            'joint3_to_joint2', 
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6'
        ]
        
        # Create goal constraints
        constraints = Constraints()
        for i, (name, value) in enumerate(zip(joint_names, joint_values)):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        request.goal_constraints.append(constraints)
        
        goal.request = request
        goal.planning_options.plan_only = not execute
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        
        # Send goal
        self.get_logger().info('Sending motion plan request...')
        send_goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=30.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
            
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        result = result_future.result().result
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Motion planning succeeded!')
            if result.planned_trajectory.joint_trajectory.points:
                num_points = len(result.planned_trajectory.joint_trajectory.points)
                self.get_logger().info(f'Trajectory has {num_points} points')
            return True
        else:
            self.get_logger().error(f'Motion planning failed with error code: {result.error_code.val}')
            return False


def main():
    rclpy.init()
    
    tester = MoveIt2Tester()
    
    # Wait for connections
    if not tester.wait_for_server():
        tester.destroy_node()
        rclpy.shutdown()
        return
        
    # Wait a bit for joint states
    import time
    for _ in range(20):  # Wait up to 2 seconds
        rclpy.spin_once(tester, timeout_sec=0.1)
        if tester.current_joint_state:
            break
    
    if not tester.current_joint_state:
        tester.get_logger().warn('No joint states received, planning from default state')
    else:
        tester.get_logger().info(f'Current joint positions: {list(tester.current_joint_state.position)}')
    
    # Test 1: Plan to home position (all zeros)
    print("\n=== Test 1: Plan to home position ===")
    home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    success = tester.plan_to_joint_goal(home_position, execute=False)
    print(f"Home position planning: {'SUCCESS' if success else 'FAILED'}")
    
    # Test 2: Plan to a different position
    print("\n=== Test 2: Plan to test position ===")
    test_position = [0.5, -0.3, 0.4, 0.0, 0.3, 0.0]  # radians
    success = tester.plan_to_joint_goal(test_position, execute=False)
    print(f"Test position planning: {'SUCCESS' if success else 'FAILED'}")
    
    print("\n=== MoveIt2 Testing Complete ===")
    print("If both tests succeeded, MoveIt2 is working correctly!")
    print("The RViz Motion Planning plugin error is a known bug in ROS2 Humble.")
    print("You can use Python/C++ APIs or command-line tools for motion planning.")
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
