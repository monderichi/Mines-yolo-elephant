#!/usr/bin/env python3
"""
Launch MoveIt 2 with joint_state_publisher_gui for myCobot 320 M5.

This provides both motion planning and manual joint control via sliders.

:author: Based on AutomaticAddison's work
:date: December 2024
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def generate_launch_description():
    """Generate launch description for MoveIt2 with joint sliders."""
    
    # Package names
    package_name_description = 'mycobot_description'
    package_name_moveit_config = 'mycobot_moveit_config'
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='mycobot_320',
        description='Name of the robot')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    def configure_setup(context):
        """Configure MoveIt and create nodes."""
        robot_name_str = LaunchConfiguration('robot_name').perform(context)
        
        # Get package paths
        pkg_share_description = FindPackageShare(package_name_description).find(package_name_description)
        pkg_share_moveit_config = FindPackageShare(package_name_moveit_config).find(package_name_moveit_config)
        
        # Construct file paths
        urdf_path = os.path.join(pkg_share_description, 'urdf', 'robots', f'{robot_name_str}.urdf.xacro')
        config_path = os.path.join(pkg_share_moveit_config, 'config', robot_name_str)
        rviz_config_path = os.path.join(pkg_share_moveit_config, 'rviz', 'move_group.rviz')
        
        # Process XACRO to get robot description
        robot_description_content = xacro.process_file(urdf_path).toxml()
        robot_description = {'robot_description': robot_description_content}
        
        # Create MoveIt configuration
        moveit_config = (
            MoveItConfigsBuilder(robot_name_str, package_name=package_name_moveit_config)
            .trajectory_execution(file_path=os.path.join(config_path, 'moveit_controllers.yaml'))
            .robot_description_semantic(file_path=os.path.join(config_path, f'{robot_name_str}.srdf'))
            .joint_limits(file_path=os.path.join(config_path, 'joint_limits.yaml'))
            .robot_description_kinematics(file_path=os.path.join(config_path, 'kinematics.yaml'))
            .planning_pipelines(
                pipelines=["ompl", "pilz_industrial_motion_planner"],
                default_planning_pipeline="ompl"
            )
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .pilz_cartesian_limits(file_path=os.path.join(config_path, 'pilz_cartesian_limits.yaml'))
            .to_moveit_configs()
        )
        
        # Robot State Publisher
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        )
        
        # Joint State Publisher GUI - for manual joint control
        joint_state_publisher_gui_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        )
        
        # MoveIt Move Group node
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim_time},
            ],
        )
        
        # RViz
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                {'use_sim_time': use_sim_time},
            ],
        )
        
        return [
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            move_group_node,
            rviz_node,
        ]

    ld = LaunchDescription()
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(OpaqueFunction(function=configure_setup))
    
    return ld
