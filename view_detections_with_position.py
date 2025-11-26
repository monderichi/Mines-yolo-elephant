#!/usr/bin/env python3
"""
View YOLO detections with 3D positions overlaid on the image
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json


class DetectionViewer(Node):
    def __init__(self):
        super().__init__('detection_viewer')
        
        self.bridge = CvBridge()
        self.latest_detections = []
        
        # Subscribe to detection image
        self.image_sub = self.create_subscription(
            Image,
            '/yolo/detection_image',
            self.image_callback,
            10
        )
        
        # Subscribe to detection info
        self.detection_sub = self.create_subscription(
            String,
            '/yolo/detections',
            self.detection_callback,
            10
        )
        
        self.get_logger().info('Detection Viewer started')
        self.get_logger().info('Press Q in the window to quit')
        
    def detection_callback(self, msg):
        """Store the latest detection data"""
        try:
            self.latest_detections = json.loads(msg.data)
        except:
            self.latest_detections = []
    
    def image_callback(self, msg):
        """Display image with 3D position overlay"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Overlay 3D positions on the image
            for det in self.latest_detections:
                if 'position_3d' in det:
                    # Get bbox center
                    if 'bbox' in det:
                        bbox = det['bbox']
                        center_x = int((bbox['x1'] + bbox['x2']) / 2)
                        center_y = int((bbox['y1'] + bbox['y2']) / 2)
                    elif 'obb' in det:
                        center_x = int(det['obb']['center_x'])
                        center_y = int(det['obb']['center_y'])
                    else:
                        continue
                    
                    # Get 3D position
                    pos = det['position_3d']
                    x, y, z = pos['x'], pos['y'], pos['z']
                    
                    # Create position text
                    pos_text = f"({x:.2f}, {y:.2f}, {z:.2f})m"
                    class_text = f"{det['class']} {det['confidence']:.2f}"
                    
                    # Draw text with background
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.6
                    thickness = 2
                    
                    # Position text
                    (w, h), _ = cv2.getTextSize(pos_text, font, font_scale, thickness)
                    cv2.rectangle(cv_image, (center_x - 5, center_y - h - 5), 
                                (center_x + w + 5, center_y + 5), (0, 0, 0), -1)
                    cv2.putText(cv_image, pos_text, (center_x, center_y), 
                              font, font_scale, (0, 255, 255), thickness)
                    
                    # Class text above
                    (w2, h2), _ = cv2.getTextSize(class_text, font, font_scale, thickness)
                    cv2.rectangle(cv_image, (center_x - 5, center_y - h - h2 - 10), 
                                (center_x + w2 + 5, center_y - h - 5), (0, 0, 0), -1)
                    cv2.putText(cv_image, class_text, (center_x, center_y - h - 5), 
                              font, font_scale, (0, 255, 0), thickness)
            
            # Display the image
            cv2.imshow('YOLO Detections with 3D Position', cv_image)
            
            # Exit on 'q' key
            key = cv2.waitKey(1)
            if key == ord('q') or key == ord('Q'):
                self.get_logger().info('Quitting...')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = DetectionViewer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
