#!/usr/bin/env python3
"""Simple MoveIt2 demo launch file for myCobot 320 M5"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Get package directories
    moveit_config_pkg = get_package_share_directory('mycobot_320_moveit2')
    
    # Load URDF via xacro
    robot_description_config = xacro.process_file(
        os.path.join(moveit_config_pkg, 'config', 'firefighter.urdf.xacro')
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Load SRDF
    robot_description_semantic_content = load_file(
        'mycobot_320_moveit2', 'config/firefighter.srdf'
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # Kinematics configuration
    kinematics_yaml = load_yaml('mycobot_320_moveit2', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # Planning configuration
    ompl_planning_yaml = load_yaml('mycobot_320_moveit2', 'config/ompl_planning.yaml') or {}
    
    # Joint limits
    joint_limits_yaml = load_yaml('mycobot_320_moveit2', 'config/joint_limits.yaml')
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    # MoveIt controllers
    moveit_controllers_yaml = load_yaml('mycobot_320_moveit2', 'config/moveit_controllers.yaml')

    # Trajectory execution
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # Planning scene monitor
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # ros2_control configuration
    ros2_controllers_yaml = load_yaml('mycobot_320_moveit2', 'config/ros2_controllers.yaml')

    # Static TF publisher for world -> base
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base'],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ros2_control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            ros2_controllers_yaml,
        ],
        output='screen',
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
    )

    # Arm controller spawner
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_group_controller', '-c', '/controller_manager'],
    )

    # Move Group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_yaml,
            trajectory_execution,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz
    rviz_config_file = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
        ],
    )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        rviz_node,
    ])
