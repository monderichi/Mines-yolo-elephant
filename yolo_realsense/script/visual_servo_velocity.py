#!/usr/bin/env python3
"""
Velocity-Based Visual Servoing for myCobot 320 M5 + RealSense D455

Uses MoveIt Servo for real-time velocity control based on ArUco marker detection.

Approach (PBVS - Position-Based Visual Servoing):
1. Detect ArUco marker, estimate 3D pose relative to camera
2. Compute position error: error = marker_position - desired_position  
3. Apply proportional control: velocity = -Kp * error
4. Publish TwistStamped to MoveIt Servo

Usage:
    # Terminal 1: Launch MoveIt + Robot
    ros2 launch mycobot_moveit_config real_robot.launch.py port:=/dev/ttyACM1
    
    # Terminal 2: Launch RealSense
    ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true
    
    # Terminal 3: Launch Servo node
    ros2 run moveit_servo servo_node --ros-args -p use_sim_time:=false
    
    # Terminal 4: Run this script
    python3 visual_servo_velocity.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import time


class VisualServoVelocity(Node):
    def __init__(self):
        super().__init__('visual_servo_velocity')
        
        # Parameters
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.03)  # 3cm marker
        self.declare_parameter('target_distance', 0.3)  # Target 30cm from camera
        self.declare_parameter('kp_linear', 0.5)  # Proportional gain linear
        self.declare_parameter('kp_angular', 0.3)  # Proportional gain angular
        self.declare_parameter('max_linear_vel', 0.08)  # Max 8cm/s
        self.declare_parameter('max_angular_vel', 0.2)  # Max 0.2 rad/s
        self.declare_parameter('deadband', 0.01)  # 1cm deadband
        self.declare_parameter('servo_frame', 'base_link')  # Frame for velocity commands
        
        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size').value
        self.target_distance = self.get_parameter('target_distance').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.deadband = self.get_parameter('deadband').value
        self.servo_frame = self.get_parameter('servo_frame').value
        
        # Calibration: Camera pose in robot base frame
        # From previous calibration: Camera is ~1.26m in Y, ~0.6m in Z from base
        # Rotation: Camera optical axis points roughly toward robot
        # R_{base_to_camera} transforms vectors FROM camera TO base
        r = R.from_euler('xyz', [-105.72, -12.74, 174.94], degrees=True)
        self.R_cam_to_base = r.as_matrix()
        self.t_cam_to_base = np.array([-0.0444, 1.2601, 0.5974])
        
        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, cv2.aruco.DetectorParameters()
        )
        
        # Camera
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_image = None
        
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', 
            self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.camera_info_callback, 10
        )
        
        # Servo status subscription
        self.servo_status = None
        self.status_sub = self.create_subscription(
            Int8, '/servo_node/status',
            self.servo_status_callback, 10
        )
        
        # Publisher for velocity commands
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10
        )
        
        # Control loop timer (50 Hz to match servo publish rate)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # State
        self.enabled = True
        self.last_detection_time = 0.0
        self.detection_timeout = 0.5  # Stop if no detection for 0.5s
        
        self.get_logger().info('Visual Servo Velocity Node initialized')
        self.get_logger().info(f'  Target distance: {self.target_distance}m')
        self.get_logger().info(f'  Kp linear: {self.kp_linear}, Kp angular: {self.kp_angular}')
        self.get_logger().info(f'  Max vel: {self.max_linear_vel} m/s')
        
    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received')
    
    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion error: {e}')
    
    def servo_status_callback(self, msg):
        self.servo_status = msg.data
        # Status codes: 0=OK, -1=Error, 1=Near singularity, etc.
        if msg.data < 0:
            self.get_logger().warn(f'Servo status: {msg.data}')
    
    def detect_marker(self):
        """Detect ArUco marker and return position in camera frame."""
        if self.latest_image is None or self.camera_matrix is None:
            return None, None
        
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        
        if ids is None:
            return None, None
        
        for i, mid in enumerate(ids.flatten()):
            if mid == self.marker_id:
                marker_corners = corners[i][0]
                half_size = self.marker_size / 2.0
                obj_points = np.array([
                    [-half_size,  half_size, 0],
                    [ half_size,  half_size, 0],
                    [ half_size, -half_size, 0],
                    [-half_size, -half_size, 0]
                ], dtype=np.float32)
                
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, marker_corners,
                    self.camera_matrix, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                
                if success:
                    # Return position and rotation
                    return tvec.flatten(), rvec.flatten()
        
        return None, None
    
    def control_loop(self):
        """Main control loop - runs at 50 Hz."""
        if not self.enabled:
            return
        
        # Detect marker
        marker_pos_cam, marker_rot_cam = self.detect_marker()
        
        # Create twist message
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.servo_frame
        
        if marker_pos_cam is None:
            # No detection - check timeout
            if time.time() - self.last_detection_time > self.detection_timeout:
                # Publish zero velocity (stop)
                self.twist_pub.publish(twist)
            return
        
        self.last_detection_time = time.time()
        
        # Marker position in camera frame: [x_cam, y_cam, z_cam]
        # Camera frame: Z forward, X right, Y down
        x_cam, y_cam, z_cam = marker_pos_cam
        
        # Desired position in camera frame: centered, at target distance
        # error = current - desired
        error_x = x_cam - 0.0  # Want marker at x=0 (centered horizontally)
        error_y = y_cam - 0.0  # Want marker at y=0 (centered vertically)
        error_z = z_cam - self.target_distance  # Want marker at target distance
        
        error_cam = np.array([error_x, error_y, error_z])
        
        # Transform error to base frame for velocity command
        # v_base = R_cam_to_base @ v_cam
        error_base = self.R_cam_to_base @ error_cam
        
        # Check if within deadband
        error_norm = np.linalg.norm(error_cam)
        if error_norm < self.deadband:
            self.get_logger().info('ALIGNED! Within deadband.')
            self.twist_pub.publish(twist)  # Zero velocity
            return
        
        # Proportional control: velocity = -Kp * error
        # Negative because we want to move toward the marker (reduce error)
        vel_linear = -self.kp_linear * error_base
        
        # Clamp velocity
        vel_magnitude = np.linalg.norm(vel_linear)
        if vel_magnitude > self.max_linear_vel:
            vel_linear = vel_linear / vel_magnitude * self.max_linear_vel
        
        # For now, no angular velocity control (keep orientation fixed)
        # Could add angular control based on marker orientation later
        
        # Populate twist message
        twist.twist.linear.x = float(vel_linear[0])
        twist.twist.linear.y = float(vel_linear[1])
        twist.twist.linear.z = float(vel_linear[2])
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        
        # Log occasionally
        if int(time.time() * 2) % 2 == 0:  # Every ~0.5s
            self.get_logger().info(
                f'Error(cam): [{error_x:.3f}, {error_y:.3f}, {error_z:.3f}] | '
                f'Vel(base): [{vel_linear[0]:.3f}, {vel_linear[1]:.3f}, {vel_linear[2]:.3f}]'
            )
        
        # Publish
        self.twist_pub.publish(twist)
        
        # Show visualization
        self.show_visualization(marker_pos_cam, error_cam)
    
    def show_visualization(self, marker_pos, error):
        """Show OpenCV visualization window."""
        if self.latest_image is None:
            return
        
        vis_img = self.latest_image.copy()
        
        # Draw marker detection
        gray = cv2.cvtColor(vis_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis_img, corners, ids)
        
        # Draw crosshair at center (target)
        h, w = vis_img.shape[:2]
        cv2.line(vis_img, (w//2-20, h//2), (w//2+20, h//2), (0, 255, 0), 2)
        cv2.line(vis_img, (w//2, h//2-20), (w//2, h//2+20), (0, 255, 0), 2)
        
        # Draw info
        cv2.putText(vis_img, f'Error: {np.linalg.norm(error):.3f}m', 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(vis_img, f'Marker Z: {marker_pos[2]:.3f}m', 
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(vis_img, f'Target: {self.target_distance:.2f}m', 
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow('Visual Servo', vis_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoVelocity()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish zero velocity before shutting down
        twist = TwistStamped()
        twist.header.stamp = node.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        node.twist_pub.publish(twist)
        
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
