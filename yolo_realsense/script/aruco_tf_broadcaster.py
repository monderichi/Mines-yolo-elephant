#!/usr/bin/env python3
"""
ArUco Marker TF Broadcaster for myCobot 320 M5 + RealSense D455
Detects ArUco markers and broadcasts their TF frames relative to the camera.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R


class ArUcoTFBroadcaster(Node):
    def __init__(self):
        super().__init__('aruco_tf_broadcaster')
        
        # Parameters
        self.declare_parameter('marker_size', 0.03)  # 3cm
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        dict_name = self.get_parameter('aruco_dictionary').get_parameter_value().string_value
        
        # ArUco Setup
        aruco_dicts = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL
        }
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dicts.get(dict_name, cv2.aruco.DICT_4X4_50))
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # CV Bridge
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        
        self.get_logger().info(f'ArUco TF Broadcaster initialized with {dict_name} and size {self.marker_size}m')

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
            
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    # Solve PnP for each marker
                    obj_points = np.array([
                        [-self.marker_size/2,  self.marker_size/2, 0],
                        [ self.marker_size/2,  self.marker_size/2, 0],
                        [ self.marker_size/2, -self.marker_size/2, 0],
                        [-self.marker_size/2, -self.marker_size/2, 0]
                    ], dtype=np.float32)
                    
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, corners[i][0],
                        self.camera_matrix, self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    
                    if success:
                        self.broadcast_tf(tvec, rvec, marker_id, msg.header.frame_id, msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f'Error in ArUco detection: {e}')

    def broadcast_tf(self, tvec, rvec, marker_id, frame_id, stamp):
        t = TransformStamped()
        
        t.header.stamp = stamp
        t.header.frame_id = frame_id
        t.child_frame_id = f'aruco_marker_{marker_id}'
        
        # Translation
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        
        # Rotation (Convert rotation vector to quaternion)
        rmat, _ = cv2.Rodrigues(rvec)
        # OpenCV camera frame to ROS optical frame is usually handled by the camera driver,
        # but here we are relative to the 'frame_id' provided by the image message.
        # For D455 'camera_color_optical_frame', Z is forward, X is right, Y is down.
        
        quat = R.from_matrix(rmat).as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
