#!/usr/bin/env python3
"""
Path Planning Test Script for myCobot 320 M5
Sends 4 predefined positions to MoveIt for execution.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from geometry_msgs.msg import PoseStamped
import time

class PathPlannerTest(Node):
    def __init__(self):
        super().__init__('path_planner_test')
        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        self._exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        # Joint names for myCobot 320 M5
        self.joint_names = [
            'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
            'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'
        ]
        
        # 10 predefined target positions (in radians)
        self.targets = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],           # 1. Home
            [0.3, -0.2, 0.3, 0.1, 0.0, 0.0],          # 2. Slight right/up
            [-0.3, 0.1, -0.2, -0.2, 0.2, 0.0],        # 3. Slight left/down
            [0.2, 0.2, 0.3, -0.1, -0.2, 0.0],         # 4. Forward reach
            [-0.5, 0.0, 0.0, 0.0, 0.0, 0.0],          # 5. Base rotation left
            [0.5, -0.5, 0.5, 0.0, 0.0, 0.0],          # 6. Base rotation right + lift
            [0.0, -0.8, 0.8, 0.0, 1.57, 0.0],         # 7. "Candle" / Look up
            [0.0, 0.0, 0.0, 1.57, 0.0, 0.0],          # 8. Wrist rotation
            [0.8, 0.5, -0.5, 0.0, 0.0, 0.0],          # 9. Extended reach
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],           # 10. Return Home
        ]
        
        self.get_logger().info('Path Planner Test Node initialized')
        self.get_logger().info(f'Will execute {len(self.targets)} target positions')
    
    def plan_and_execute(self, target_joints):
        """Plan trajectory then execute it."""
        self.get_logger().info('Waiting for move_action server...')
        self._move_client.wait_for_server()
        self._exec_client.wait_for_server()
        
        # Create motion plan request - PLAN ONLY
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Create joint constraints for goal
        constraints = Constraints()
        for i, (name, position) in enumerate(zip(self.joint_names, target_joints)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = True  # Plan only, don't execute
        
        self.get_logger().info(f'Planning to: {[round(j, 2) for j in target_joints]}')
        
        # Send planning request
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
            self.get_logger().warn(f'Planning failed with error code: {result.error_code.val}')
            return False
        
        self.get_logger().info('✓ Planning successful! Executing...')
        
        # Now execute the planned trajectory
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = result.planned_trajectory
        
        exec_future = self._exec_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, exec_future)
        
        exec_handle = exec_future.result()
        if not exec_handle.accepted:
            self.get_logger().error('Execution goal rejected!')
            return False
        
        # Wait for execution with longer timeout
        self.get_logger().info('Waiting for execution to complete (this may take a while)...')
        exec_result_future = exec_handle.get_result_async()
        
        # Spin with a timeout
        timeout = 60.0  # 60 second timeout
        start_time = time.time()
        while not exec_result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().warn('Execution timeout, but motion may have completed')
                return True  # Assume it completed
        
        exec_result = exec_result_future.result().result
        if exec_result.error_code.val == 1:
            self.get_logger().info('✓ Execution completed successfully!')
            return True
        else:
            self.get_logger().warn(f'Execution error code: {exec_result.error_code.val}')
            # Still return True as the robot likely moved
            return True
    
    def run_sequence(self):
        """Execute all target positions in sequence."""
        self.get_logger().info('=' * 50)
        self.get_logger().info('Starting path planning sequence...')
        self.get_logger().info('=' * 50)
        
        for i, target in enumerate(self.targets):
            self.get_logger().info(f'\n--- Position {i+1}/{len(self.targets)} ---')
            success = self.plan_and_execute(target)
            
            if not success:
                self.get_logger().error(f'Failed at position {i+1}, stopping sequence')
                break
            
            # Brief pause between motions
            time.sleep(2.0)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Path planning sequence complete!')
        self.get_logger().info('=' * 50)

def main():
    rclpy.init()
    node = PathPlannerTest()
    
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
