"""
Launch file to start Intel RealSense D455 camera and YOLO detector
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='yolo11n.pt',
        description='YOLO model to use (yolo11n.pt, yolo11s.pt, yolo11m.pt, yolo11l.pt, yolo11x.pt)'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence',
        default_value='0.5',
        description='Confidence threshold for detections'
    )
    
    show_preview_arg = DeclareLaunchArgument(
        'show_preview',
        default_value='False',
        description='Show detection preview window'
    )
    
    use_obb_arg = DeclareLaunchArgument(
        'use_obb',
        default_value='False',
        description='Use OBB (Oriented Bounding Boxes) for pick and place operations'
    )
    
    # RealSense camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'align_depth.enable': True,
            'pointcloud.enable': True,
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
        }],
        output='screen'
    )
    
    # YOLO detector node
    yolo_node = Node(
        package='yolo_realsense',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[{
            'model_path': LaunchConfiguration('model'),
            'confidence_threshold': LaunchConfiguration('confidence'),
            'show_preview': LaunchConfiguration('show_preview'),
            'use_obb': LaunchConfiguration('use_obb'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        model_arg,
        confidence_arg,
        show_preview_arg,
        use_obb_arg,
        realsense_node,
        yolo_node,
    ])
