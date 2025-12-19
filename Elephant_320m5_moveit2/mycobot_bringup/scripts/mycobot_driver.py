#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import time
import traceback
from pymycobot.mycobot320 import MyCobot320

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class MyCobot320Driver(Node):
    def __init__(self):
        super().__init__('mycobot_320_driver')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.cb_group = ReentrantCallbackGroup()
        
        self.joint_names = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
                           'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
        
        self.get_logger().info(f'Connecting to myCobot on {port}...')
        try:
            self.mc = MyCobot320(port, str(baud))
            time.sleep(0.5)
            self.mc.power_on()
            angles = self.mc.get_angles()
            self.get_logger().info(f'Connected! Angles: {angles}')
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')
        
        # Publisher for joint states (runs on timer)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Action server for trajectory execution (long running)
        self._action_server = ActionServer(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.execute_trajectory,
            callback_group=self.cb_group)
        
        self.timer = self.create_timer(0.05, self.publish_joint_states, callback_group=self.cb_group)
        self.get_logger().info('Driver ready!')
    
    def publish_joint_states(self):
        try:
            angles = self.mc.get_angles()
            if angles and len(angles) == 6:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names
                msg.position = [math.radians(a) for a in angles]
                msg.velocity = []
                msg.effort = []
                self.joint_state_pub.publish(msg)
        except: pass
    
    def execute_trajectory(self, goal_handle):
        self.get_logger().info('Received trajectory request!')
        try:
            trajectory = goal_handle.request.trajectory
            num_points = len(trajectory.points)
            self.get_logger().info(f'Trajectory has {num_points} points')
            
            # Execute trajectory with 20Hz interpolation for smooth motion
            start_time = time.time()
            if num_points > 0:
                total_duration = trajectory.points[-1].time_from_start.sec + trajectory.points[-1].time_from_start.nanosec / 1e9
            else:
                total_duration = 0.0

            # Target frequency 20Hz
            rate_hz = 20.0
            dt = 1.0 / rate_hz
            next_tick = start_time + dt
            
            # Main execution loop
            while True:
                now = time.time()
                t_from_start = now - start_time
                
                if t_from_start >= total_duration:
                    break
                    
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal cancel requested!')
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()
                
                # Find the segment [p_prev, p_next] for current time
                p_prev = trajectory.points[0]
                p_next = trajectory.points[-1]
                
                for pt in trajectory.points:
                    pt_time = pt.time_from_start.sec + pt.time_from_start.nanosec / 1e9
                    if pt_time <= t_from_start:
                        p_prev = pt
                    else:
                        p_next = pt
                        break
                
                # Interpolate positions
                t1 = p_prev.time_from_start.sec + p_prev.time_from_start.nanosec / 1e9
                t2 = p_next.time_from_start.sec + p_next.time_from_start.nanosec / 1e9
                
                cmd_angles = []
                for j in range(len(p_prev.positions)):
                    val1 = p_prev.positions[j]
                    val2 = p_next.positions[j]
                    
                    if t2 - t1 < 1e-6:
                        pos = val1
                    else:
                        ratio = (t_from_start - t1) / (t2 - t1)
                        pos = val1 + ratio * (val2 - val1)
                    
                    cmd_angles.append(math.degrees(pos))
                
                # Send interpolated command
                self.mc.send_angles(cmd_angles, 95)
                
                # Maintain loop rate
                sleep_time = next_tick - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                # Ensure we don't fall behind if one iteration is slow
                now_after = time.time()
                if now_after > next_tick:
                     # Skip ticks to catch up
                     next_tick = now_after + dt
                else:
                     next_tick += dt
            
            # Ensure final position is exactly reached
            if num_points > 0:
                final_point = trajectory.points[-1]
                final_angles = [math.degrees(p) for p in final_point.positions]
                self.mc.send_angles(final_angles, 95)
            
            # Wait a bit for the robot to physically settle
            time.sleep(0.5)
            
            goal_handle.succeed()
            self.get_logger().info('Trajectory completed!')
            return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)
        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')
            self.get_logger().error(traceback.format_exc())
            goal_handle.abort()
            return FollowJointTrajectory.Result(error_code=-1)

def main():
    rclpy.init()
    node = MyCobot320Driver()
    # Use MultiThreadedExecutor to allow action server and publisher to run in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
