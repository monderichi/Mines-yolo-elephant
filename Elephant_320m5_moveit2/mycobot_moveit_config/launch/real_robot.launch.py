#!/usr/bin/env python3
"""
Launch MoveIt2 with real myCobot 320 M5 hardware.

This launch file:
1. Starts the mycobot_driver node (bridges pymycobot with ROS2)
2. Starts robot_state_publisher
3. Starts move_group for motion planning
4. Starts RViz for visualization

:author: Generated for myCobot 320 M5 MoveIt2 control
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def generate_launch_description():
    """Generate launch description for real robot control."""
    
    # Package names
    package_name_description = 'mycobot_description'
    package_name_moveit_config = 'mycobot_moveit_config'
    package_name_bringup = 'mycobot_bringup'
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    port = LaunchConfiguration('port')
    
    # Declare launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='mycobot_320',
        description='Name of the robot')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation clock if true')
    
    declare_port_cmd = DeclareLaunchArgument(
        name='port',
        default_value='/dev/ttyACM0',
        description='Serial port for myCobot')

    def configure_setup(context):
        """Configure MoveIt and create nodes."""
        robot_name_str = LaunchConfiguration('robot_name').perform(context)
        port_str = LaunchConfiguration('port').perform(context)
        
        # Get package paths
        pkg_share_description = FindPackageShare(package_name_description).find(package_name_description)
        pkg_share_moveit_config = FindPackageShare(package_name_moveit_config).find(package_name_moveit_config)
        pkg_share_bringup = FindPackageShare(package_name_bringup).find(package_name_bringup)
        
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
        
        # myCobot Driver node - connects to real robot
        mycobot_driver_node = Node(
            package=package_name_bringup,
            executable='mycobot_driver.py',
            name='mycobot_driver',
            output='screen',
            parameters=[
                {'port': port_str},
                {'baud': 115200},
                {'publish_rate': 20.0},
                {'use_sim_time': use_sim_time}
            ],
        )
        
        # Robot State Publisher
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
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
        
        # Static TF: ArUco marker attached to link6 (end effector)
        # The marker is a sticker at the EE - 180° rotation on Z to align X,Y axes
        ee_marker_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ee_aruco_marker_tf',
            arguments=[
                '0.0', '0.0', '0.0',  # Small offset from link6 center (20mm forward)
                '0.0', '0.0', '1.0', '0.0',  # 180° rotation around Z axis (quat xyzw)
                'link6', 'ee_aruco_marker'  # Parent: link6, Child: ee_aruco_marker
            ]
        )
        
        # Calibrated Camera Transform (Robust Eye-to-Hand)
        camera_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_tf',
            arguments=[
                '0.077347', '0.750052', '0.298595',
                '0.086084', '0.019093', '-0.692978', '0.715546',
                'base_link', 'camera_link'
            ]
        )
        
        return [
            mycobot_driver_node,
            robot_state_publisher_node,
            move_group_node,
            rviz_node,
            ee_marker_tf,
            camera_tf,
        ]

    ld = LaunchDescription()
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_port_cmd)
    ld.add_action(OpaqueFunction(function=configure_setup))
    
    return ld
