#!/usr/bin/env python3
"""
Combined launch file for YOLO detection + MoveIt2 + Pick Controller.
Launches everything needed for camera-guided pick-and-place.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description."""
    
    # Launch arguments
    declare_target_class = DeclareLaunchArgument(
        'target_class',
        default_value='mouse',
        description='Object class to track and pick'
    )
    
    declare_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for myCobot'
    )
    
    declare_show_preview = DeclareLaunchArgument(
        'show_preview',
        default_value='True',
        description='Show YOLO detection preview'
    )

    declare_enable_controller = DeclareLaunchArgument(
        'enable_controller',
        default_value='True',
        description='Enable pick controller node'
    )

    def setup_nodes(context):
        target_class = LaunchConfiguration('target_class').perform(context)
        port = LaunchConfiguration('port').perform(context)
        show_preview = LaunchConfiguration('show_preview').perform(context)
        enable_controller = LaunchConfiguration('enable_controller').perform(context)
        
        # Get package directories
        moveit_config_pkg = get_package_share_directory('mycobot_moveit_config')
        yolo_pkg = get_package_share_directory('yolo_realsense')
        
        # Include MoveIt real robot launch
        moveit_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_pkg, 'launch', 'real_robot.launch.py')
            ),
            launch_arguments={
                'port': port,
            }.items()
        )
        
        # Include YOLO RealSense launch
        yolo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(yolo_pkg, 'launch', 'yolo_realsense.launch.py')
            ),
            launch_arguments={
                'show_preview': show_preview,
            }.items()
        )
        
        # Static transform: camera_link -> base_link
        # Camera is 90cm to the LEFT (Y+) and 2cm above (Z+) robot base
        # Camera optical frame looks toward robot
        static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_tf',
            arguments=[
                '--x', '0.0',
                '--y', '0.90',  # 90cm left
                '--z', '0.02',  # 2cm above
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', str(3.14159/2),  # 90 degrees - camera faces robot
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link'
            ]
        )
        
        nodes = [
            moveit_launch,
            yolo_launch,
            static_tf_node,
        ]

        if enable_controller == 'True':
            # Pick controller node
            pick_controller_node = Node(
                package='yolo_realsense',
                executable='pick_controller.py',
                name='pick_controller',
                output='screen',
                parameters=[{
                    'target_class': target_class,
                    'approach_height': 0.05,  # 5cm above object
                    'min_confidence': 0.5,
                }]
            )
            nodes.append(pick_controller_node)
        
        return nodes
    
    return LaunchDescription([
        declare_target_class,
        declare_port,
        declare_show_preview,
        declare_enable_controller,
        OpaqueFunction(function=setup_nodes),
    ])
