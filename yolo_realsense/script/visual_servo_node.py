#!/usr/bin/env python3
"""
Visual Servoing Node for myCobot 320 M5 + RealSense D455

This node:
1. Detects ArUco marker in camera image
2. Computes error from desired position (e.g., marker centered in camera)
3. Publishes TwistStamped commands to MoveIt Servo
4. Robot smoothly tracks the marker in real-time

Usage:
    ros2 run yolo_realsense visual_servo_node.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
import numpy as np


class VisualServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')
        
        # ------ Parameters ------
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.03)  # 30mm
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('servo_command_topic', '/servo_node/delta_twist_cmds')
        self.declare_parameter('target_distance', 0.5)  # Target distance to marker (m)
        self.declare_parameter('deadband_linear', 0.01)  # Stop if error < 1cm
        self.declare_parameter('deadband_angular', 0.02)  # Stop if angular error < 0.02 rad
        self.declare_parameter('gain_linear', 0.5)  # Proportional gain for linear velocity
        self.declare_parameter('gain_angular', 0.3)  # Proportional gain for angular velocity
        self.declare_parameter('max_linear_vel', 0.1)  # m/s
        self.declare_parameter('max_angular_vel', 0.3)  # rad/s
        
        # Get parameters
        aruco_dict_name = self.get_parameter('aruco_dict').get_parameter_value().string_value
        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        servo_topic = self.get_parameter('servo_command_topic').get_parameter_value().string_value
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.deadband_linear = self.get_parameter('deadband_linear').get_parameter_value().double_value
        self.deadband_angular = self.get_parameter('deadband_angular').get_parameter_value().double_value
        self.gain_linear = self.get_parameter('gain_linear').get_parameter_value().double_value
        self.gain_angular = self.get_parameter('gain_angular').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        
        # ArUco detector
        aruco_dicts = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
        }
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            aruco_dicts.get(aruco_dict_name, cv2.aruco.DICT_4X4_50))
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Camera
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_width = 0
        self.image_height = 0
        
        # State
        self.marker_visible = False
        self.last_detection_time = self.get_clock().now()
        self.servo_active = True
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        
        # Publisher for servo commands
        self.twist_pub = self.create_publisher(TwistStamped, servo_topic, 10)
        
        # Timer for control loop (30 Hz)
        self.control_timer = self.create_timer(0.033, self.control_loop)
        
        # Store latest detection
        self.latest_tvec = None
        self.latest_rvec = None
        self.latest_center_error = None
        
        self.get_logger().info('Visual Servo Node initialized')
        self.get_logger().info(f'  ArUco: {aruco_dict_name}, ID: {self.marker_id}')
        self.get_logger().info(f'  Target distance: {self.target_distance}m')
        self.get_logger().info(f'  Publishing to: {servo_topic}')
    
    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received')
    
    def image_callback(self, msg):
        """Process image and detect ArUco marker."""
        if self.camera_matrix is None:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return
        
        self.image_height, self.image_width = cv_image.shape[:2]
        
        # Detect ArUco
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        
        if ids is None:
            self.marker_visible = False
            self.latest_tvec = None
            self.latest_rvec = None
            return
        
        # Find our marker
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == self.marker_id:
                marker_corners = corners[i][0]
                
                # Compute center in image
                center_x = np.mean(marker_corners[:, 0])
                center_y = np.mean(marker_corners[:, 1])
                
                # Pixel error from image center
                error_x = center_x - self.image_width / 2
                error_y = center_y - self.image_height / 2
                self.latest_center_error = (error_x, error_y)
                
                # Pose estimation
                half_size = self.marker_size / 2.0
                obj_points = np.array([
                    [-half_size, half_size, 0],
                    [half_size, half_size, 0],
                    [half_size, -half_size, 0],
                    [-half_size, -half_size, 0]
                ], dtype=np.float32)
                
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, marker_corners,
                    self.camera_matrix, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                
                if success:
                    self.latest_tvec = tvec.flatten()
                    self.latest_rvec = rvec.flatten()
                    self.marker_visible = True
                    self.last_detection_time = self.get_clock().now()
                return
        
        # Marker ID not found
        self.marker_visible = False
    
    def control_loop(self):
        """Compute and publish servo commands based on marker detection."""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        
        # Check if marker is visible
        if not self.marker_visible or self.latest_tvec is None:
            # Stop the robot if marker lost for too long
            time_since_detection = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
            if time_since_detection > 0.5:  # 500ms timeout
                # Publish zero velocity
                self.twist_pub.publish(twist)
            return
        
        # Extract marker position in camera frame
        # tvec: [x, y, z] where z is distance
        marker_x = self.latest_tvec[0]  # Left/right in camera frame
        marker_y = self.latest_tvec[1]  # Up/down in camera frame
        marker_z = self.latest_tvec[2]  # Distance (depth)
        
        # Compute errors
        # Goal: marker at camera center (x=0, y=0) and at target distance
        error_x = marker_x  # Lateral error (should be 0)
        error_y = marker_y  # Vertical error (should be 0)
        error_z = marker_z - self.target_distance  # Distance error
        
        # Apply deadband
        if abs(error_x) < self.deadband_linear:
            error_x = 0.0
        if abs(error_y) < self.deadband_linear:
            error_y = 0.0
        if abs(error_z) < self.deadband_linear:
            error_z = 0.0
        
        # Compute velocity commands (proportional control)
        # Camera frame to robot EE frame mapping:
        # Assuming camera is roughly aligned with EE looking forward:
        # Camera X (right) -> EE Y (left, so negate)
        # Camera Y (down) -> EE Z (up, so negate)
        # Camera Z (forward) -> EE X (forward)
        
        vx = -self.gain_linear * error_z  # Move forward/backward
        vy = -self.gain_linear * error_x  # Move left/right
        vz = -self.gain_linear * error_y  # Move up/down
        
        # Clamp velocities
        vx = np.clip(vx, -self.max_linear_vel, self.max_linear_vel)
        vy = np.clip(vy, -self.max_linear_vel, self.max_linear_vel)
        vz = np.clip(vz, -self.max_linear_vel, self.max_linear_vel)
        
        # Set twist
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = vz
        
        # No rotation for now
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        
        # Publish
        self.twist_pub.publish(twist)
        
        # Log status periodically
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % 30 == 0:  # Every ~1 second
            self.get_logger().info(
                f'Marker at ({marker_x*1000:.1f}, {marker_y*1000:.1f}, {marker_z*1000:.1f})mm, '
                f'cmd=({vx:.3f}, {vy:.3f}, {vz:.3f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
