"""
Visual Servoing Launch File for myCobot 320 M5

Launches:
1. MoveIt Servo node (for real-time velocity commands)
2. Visual Servo Velocity Node (ArUco detection + PBVS control loop)

Prerequisites:
- MoveIt + Robot already running: ros2 launch mycobot_moveit_config real_robot.launch.py
- RealSense camera already running: ros2 launch realsense2_camera rs_launch.py

Usage:
    ros2 launch yolo_realsense visual_servo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # Package names
    package_name_description = 'mycobot_description'
    package_name_moveit_config = 'mycobot_moveit_config'
    
    # Declare arguments
    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='0.3',
        description='Target distance from marker (meters)'
    )
    
    kp_linear_arg = DeclareLaunchArgument(
        'kp_linear',
        default_value='0.5',
        description='Proportional gain for linear velocity'
    )
    
    max_vel_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='0.08',
        description='Maximum linear velocity (m/s)'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='mycobot_320',
        description='Name of the robot'
    )
    
    def configure_setup(context):
        """Configure MoveIt Servo and visual servo nodes."""
        robot_name_str = LaunchConfiguration('robot_name').perform(context)
        
        # Get package paths
        pkg_share_description = FindPackageShare(package_name_description).find(package_name_description)
        pkg_share_moveit_config = FindPackageShare(package_name_moveit_config).find(package_name_moveit_config)
        pkg_share_yolo = get_package_share_directory('yolo_realsense')
        
        # Construct file paths
        urdf_path = os.path.join(pkg_share_description, 'urdf', 'robots', f'{robot_name_str}.urdf.xacro')
        config_path = os.path.join(pkg_share_moveit_config, 'config', robot_name_str)
        servo_config_path = os.path.join(pkg_share_yolo, 'config', 'servo_config.yaml')
        
        # Process XACRO to get robot description
        robot_description_content = xacro.process_file(urdf_path).toxml()
        robot_description = {'robot_description': robot_description_content}
        
        # Get SRDF content
        srdf_path = os.path.join(config_path, f'{robot_name_str}.srdf')
        with open(srdf_path, 'r') as f:
            robot_description_semantic = {'robot_description_semantic': f.read()}
        
        # Get kinematics config
        kinematics_path = os.path.join(config_path, 'kinematics.yaml')
        
        # Load kinematics config as dict
        import yaml
        with open(kinematics_path, 'r') as f:
            kinematics_config = yaml.safe_load(f)
        
        # MoveIt Servo Node - needs robot_description and robot_description_semantic
        servo_node = Node(
            package='moveit_servo',
            executable='servo_node',
            name='servo_node',
            output='screen',
            parameters=[
                servo_config_path,
                robot_description,
                robot_description_semantic,
                {'robot_description_kinematics': kinematics_config},
            ],
        )
        
        # Visual Servo Velocity Node
        visual_servo_node = Node(
            package='yolo_realsense',
            executable='visual_servo_velocity.py',
            name='visual_servo_velocity',
            output='screen',
            parameters=[{
                'target_distance': float(LaunchConfiguration('target_distance').perform(context)),
                'kp_linear': float(LaunchConfiguration('kp_linear').perform(context)),
                'max_linear_vel': float(LaunchConfiguration('max_linear_vel').perform(context)),
                'marker_id': 0,
                'marker_size': 0.03,
            }]
        )
        
        return [servo_node, visual_servo_node]
    
    return LaunchDescription([
        target_distance_arg,
        kp_linear_arg,
        max_vel_arg,
        robot_name_arg,
        OpaqueFunction(function=configure_setup),
    ])
