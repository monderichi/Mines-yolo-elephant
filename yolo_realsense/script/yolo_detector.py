#!/usr/bin/env python3
"""
YOLO11 Object Detection Node for Intel RealSense D455
Subscribes to RealSense camera images and publishes detected objects
Supports both regular bounding boxes and OBB (Oriented Bounding Boxes) for pick and place
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import json
import math
from scipy.spatial.transform import Rotation as R

# Import custom messages (will be generated after building)
try:
    from yolo_realsense.msg import OBBDetection, OBBDetectionArray
except ImportError:
    OBBDetection = None
    OBBDetectionArray = None


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Declare parameters
        self.declare_parameter('model_path', 'yolo11n.pt')  # Generic pretrained model
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('show_preview', False)
        self.declare_parameter('use_obb', False)  # Enable OBB mode for pick and place
        
        # Get parameters
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.iou_threshold = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.show_preview = self.get_parameter('show_preview').get_parameter_value().bool_value
        self.use_obb = self.get_parameter('use_obb').get_parameter_value().bool_value
        
        # Initialize YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        if self.use_obb:
            self.get_logger().info('OBB mode enabled for oriented bounding boxes')
        self.model = YOLO(model_path)
        self.get_logger().info('YOLO model loaded successfully')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Camera intrinsics (will be populated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Average dimensions for common objects (height, width, depth) in meters
        self.class_dimensions = {
            'person': np.array([1.75, 0.60, 0.60]),
            'bicycle': np.array([1.40, 0.70, 1.80]),
            'car': np.array([1.52, 1.64, 3.85]),
            'motorcycle': np.array([1.50, 0.90, 2.20]),
            'bus': np.array([3.07, 2.63, 11.17]),
            'truck': np.array([3.07, 2.63, 11.17]),
            'bottle': np.array([0.25, 0.10, 0.10]),
            'cup': np.array([0.10, 0.08, 0.08]),
            'chair': np.array([0.80, 0.60, 0.60]),
            'couch': np.array([0.80, 0.85, 2.00]),
            'potted plant': np.array([0.80, 0.40, 0.40]),
            'bed': np.array([0.60, 1.50, 2.00]),
            'dining table': np.array([0.75, 1.20, 1.20]),
            'tv': np.array([0., 0.15, 1.20]),
            'laptop': np.array([0.02, 0.25, 0.35]),
            'mouse': np.array([0.03, 0.06, 0.10]),
            'keyboard': np.array([0.03, 0.15, 0.45]),
            'cell phone': np.array([0.15, 0.08, 0.01]),
            'book': np.array([0.03, 0.20, 0.15]),
            'clock': np.array([0.20, 0.20, 0.05]),
            'vase': np.array([0.30, 0.15, 0.15]),
            'scissors': np.array([0.02, 0.15, 0.05]),
            'teddy bear': np.array([0.40, 0.30, 0.30]),
            'backpack': np.array([0.50, 0.35, 0.20]),
            'handbag': np.array([0.30, 0.40, 0.15]),
            'suitcase': np.array([0.70, 0.50, 0.25]),
            'default': np.array([0.30, 0.30, 0.20])  # Default for unknown objects
        }
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # RealSense color image topic
            self.image_callback,
            qos_profile_sensor_data
        )
        
        # Subscribe to depth image for 3D coordinates
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            qos_profile_sensor_data
        )
        
        # Subscribe to camera info
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )
        
        # Store latest depth image
        self.latest_depth = None
        
        # Create publishers
        self.detection_image_pub = self.create_publisher(
            Image,
            '/yolo/detection_image',
            10
        )
        
        self.detection_info_pub = self.create_publisher(
            String,
            '/yolo/detections',
            10
        )
        
        # 3D Marker publisher for RViz
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/yolo/markers',
            10
        )
        
        # OBB publishers (only if OBB mode is enabled and messages are available)
        if self.use_obb and OBBDetectionArray is not None:
            self.obb_detection_pub = self.create_publisher(
                OBBDetectionArray,
                '/yolo/obb_detections',
                10
            )
            self.get_logger().info('Publishing OBB detections to /yolo/obb_detections')
        
        self.get_logger().info('YOLO Detector Node initialized')
        self.get_logger().info(f'Subscribing to: /camera/camera/color/image_raw')
        self.get_logger().info(f'Publishing to: /yolo/detection_image and /yolo/detections')
    
    def camera_info_callback(self, msg):
        """Store camera intrinsics for 3D coordinate calculation"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received')
    
    def depth_callback(self, msg):
        """Store the latest depth image"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {str(e)}')
        
    def get_3d_coordinates(self, x, y, bbox=None):
        """
        Convert 2D image coordinates to 3D world coordinates using depth.
        Uses area sampling to handle missing depth values.
        
        Args:
            x, y: Center coordinates in color image
            bbox: Optional bounding box [x1, y1, x2, y2] for area sampling
            
        Returns: (x, y, z) in meters, or None if depth not available
        """
        if self.latest_depth is None or self.camera_matrix is None:
            return None
        
        # Get depth image dimensions
        depth_height, depth_width = self.latest_depth.shape
        
        # Scale coordinates from color image to depth image
        # Color is 1280x720, depth is 848x480
        scale_x = depth_width / 1280
        scale_y = depth_height / 720
        
        depth_center_x = int(x * scale_x)
        depth_center_y = int(y * scale_y)
        
        # Define search area (try center first, then expand)
        search_radius = 20  # pixels in depth image (increased for reliability)
        
        # Try to get depth from center point first
        depth_value = 0
        if 0 <= depth_center_x < depth_width and 0 <= depth_center_y < depth_height:
            depth_value = self.latest_depth[depth_center_y, depth_center_x]
        
        # If center is invalid, sample area and find valid depth
        if depth_value == 0:
            valid_depths = []
            for dy in range(-search_radius, search_radius + 1, 2):
                for dx in range(-search_radius, search_radius + 1, 2):
                    px = depth_center_x + dx
                    py = depth_center_y + dy
                    if 0 <= px < depth_width and 0 <= py < depth_height:
                        d = self.latest_depth[py, px]
                        if d > 0:
                            valid_depths.append(d)
            
            if valid_depths:
                # Use median to avoid outliers
                depth_value = int(np.median(valid_depths))
        
        if depth_value == 0:
            return None
        
        # Convert to meters
        z = depth_value / 1000.0
        
        # Get camera intrinsics (these are for color camera, so use original x,y)
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Convert to 3D coordinates using color camera coordinates
        x_3d = (x - cx) * z / fx
        y_3d = (y - cy) * z / fy
        z_3d = z
        
        return (x_3d, y_3d, z_3d)
    
    def rotation_to_quaternion(self, rotation_rad):
        """
        Convert 2D rotation angle to quaternion (rotation around Z axis)
        rotation_rad: rotation angle in radians
        Returns: Quaternion (x, y, z, w)
        """
        # Create rotation around Z axis
        rot = R.from_euler('z', rotation_rad)
        quat = rot.as_quat()  # Returns [x, y, z, w]
        return quat
        
    def calculate_real_dimensions(self, class_name, width_px, height_px, depth_m):
        """
        Calculate real-world dimensions (width, height, depth) in meters.
        Uses predefined dimensions scaled by 2D bounding box size.
        
        Args:
            class_name: Name of the detected class
            width_px: Width of 2D bounding box in pixels
            height_px: Height of 2D bounding box in pixels
            depth_m: Distance to object in meters
            
        Returns:
            tuple: (width_m, height_m, depth_m) dimensions in meters
        """
        if self.camera_matrix is None or depth_m is None:
            return None, None, None
        
        # Get base dimensions for this class
        class_lower = class_name.lower()
        if class_lower in self.class_dimensions:
            base_dims = self.class_dimensions[class_lower].copy()
        else:
            base_dims = self.class_dimensions['default'].copy()
        
        # Calculate the expected pixel size based on base dimensions and depth
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        
        # Expected pixel sizes if object were at this depth
        expected_width_px = (base_dims[1] * fx) / depth_m
        expected_height_px = (base_dims[0] * fy) / depth_m
        
        # Calculate scale factor based on actual vs expected size
        scale_width = width_px / expected_width_px if expected_width_px > 0 else 1.0
        scale_height = height_px / expected_height_px if expected_height_px > 0 else 1.0
        
        # Use average scale to maintain proportions
        scale = (scale_width + scale_height) / 2.0
        
        # Apply scale to all dimensions
        height_m = base_dims[0] * scale
        width_m = base_dims[1] * scale
        depth_m_obj = base_dims[2] * scale  # Object thickness/depth
        
        return width_m, height_m, depth_m_obj

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO inference
            results = self.model(
                cv_image,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                verbose=False
            )
            
            # Get the first result
            result = results[0]
            
            # Prepare detection info
            detections = []
            obb_detections_list = []
            
            # Draw bounding boxes and labels
            annotated_frame = result.plot()
            
            # Process detections based on mode
            if self.use_obb and hasattr(result, 'obb') and result.obb is not None:
                # OBB mode - extract oriented bounding boxes
                for obb in result.obb:
                    class_id = int(obb.cls[0])
                    confidence = float(obb.conf[0])
                    class_name = result.names[class_id]
                    
                    # Get OBB parameters
                    # obb.xywhr returns [center_x, center_y, width, height, rotation]
                    xywhr = obb.xywhr[0].cpu().numpy()
                    center_x = float(xywhr[0])
                    center_y = float(xywhr[1])
                    width = float(xywhr[2])
                    height = float(xywhr[3])
                    rotation = float(xywhr[4])  # in radians
                    
                    # Get 3D coordinates
                    coords_3d = self.get_3d_coordinates(center_x, center_y)
                    
                    detection = {
                        'class': class_name,
                        'confidence': confidence,
                        'obb': {
                            'center_x': center_x,
                            'center_y': center_y,
                            'width': width,
                            'height': height,
                            'rotation': rotation,
                            'rotation_deg': math.degrees(rotation)
                        }
                    }
                    
                    if coords_3d is not None:
                        detection['position_3d'] = {
                            'x': coords_3d[0],
                            'y': coords_3d[1],
                            'z': coords_3d[2]
                        }
                        
                        # Calculate real dimensions
                        real_w, real_h, real_d = self.calculate_real_dimensions(class_name, width, height, coords_3d[2])
                        if real_w is not None:
                            detection['dimensions'] = {
                                'width_m': real_w,
                                'height_m': real_h,
                                'depth_m': real_d
                            }
                    
                    detections.append(detection)
                    
                    # Create OBB message if available
                    if OBBDetection is not None and coords_3d is not None:
                        obb_msg = OBBDetection()
                        obb_msg.header = msg.header
                        obb_msg.class_name = class_name
                        obb_msg.confidence = confidence
                        obb_msg.center_x = center_x
                        obb_msg.center_y = center_y
                        obb_msg.width = width
                        obb_msg.height = height
                        obb_msg.rotation = rotation
                        
                        # 3D position
                        obb_msg.position = Point()
                        obb_msg.position.x = coords_3d[0]
                        obb_msg.position.y = coords_3d[1]
                        obb_msg.position.z = coords_3d[2]
                        
                        # Orientation as quaternion
                        quat = self.rotation_to_quaternion(rotation)
                        obb_msg.orientation = Quaternion()
                        obb_msg.orientation.x = float(quat[0])
                        obb_msg.orientation.y = float(quat[1])
                        obb_msg.orientation.z = float(quat[2])
                        obb_msg.orientation.w = float(quat[3])
                        
                        obb_msg.depth = coords_3d[2]
                        obb_msg.has_depth = True
                        
                        obb_detections_list.append(obb_msg)
                
            else:
                # Regular bounding box mode
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    bbox = box.xyxy[0].tolist()  # [x1, y1, x2, y2]
                    class_name = result.names[class_id]
                    
                    # Calculate center for 3D coordinates
                    center_x = (bbox[0] + bbox[2]) / 2
                    center_y = (bbox[1] + bbox[3]) / 2
                    
                    detection = {
                        'class': class_name,
                        'confidence': confidence,
                        'bbox': {
                            'x1': bbox[0],
                            'y1': bbox[1],
                            'x2': bbox[2],
                            'y2': bbox[3]
                        }
                    }
                    
                    # Add 3D coordinates if available
                    coords_3d = self.get_3d_coordinates(center_x, center_y)
                    if coords_3d is not None:
                        detection['position_3d'] = {
                            'x': coords_3d[0],
                            'y': coords_3d[1],
                            'z': coords_3d[2]
                        }
                        
                        # Calculate real dimensions
                        width_px = bbox[2] - bbox[0]
                        height_px = bbox[3] - bbox[1]
                        real_w, real_h, real_d = self.calculate_real_dimensions(class_name, width_px, height_px, coords_3d[2])
                        if real_w is not None:
                            detection['dimensions'] = {
                                'width_m': real_w,
                                'height_m': real_h,
                                'depth_m': real_d
                            }
                    
                    detections.append(detection)
            
            # Create and publish markers
            marker_array = MarkerArray()
            marker_id = 0
            for det in detections:
                if 'position_3d' in det:
                    # Create box marker
                    marker = self.create_marker(det, marker_id, msg.header.frame_id)
                    marker_array.markers.append(marker)
                    marker_id += 1
                    
                    # Create text marker
                    text_marker = self.create_text_marker(det, marker_id, msg.header.frame_id)
                    marker_array.markers.append(text_marker)
                    marker_id += 1
            
            if marker_array.markers:
                self.marker_pub.publish(marker_array)

            # Publish detection info as JSON
            if detections:
                detection_msg = String()
                detection_msg.data = json.dumps(detections, indent=2)
                self.detection_info_pub.publish(detection_msg)
                
                self.get_logger().info(
                    f'Detected {len(detections)} objects: {[d["class"] for d in detections]}'
                )
            
            # Publish OBB detections if available
            if obb_detections_list and OBBDetectionArray is not None:
                obb_array_msg = OBBDetectionArray()
                obb_array_msg.header = msg.header
                obb_array_msg.detections = obb_detections_list
                self.obb_detection_pub.publish(obb_array_msg)
            
            # Publish annotated image
            # Draw 3D boxes on the image
            for det in detections:
                self.draw_3d_box(annotated_frame, det)

            detection_image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            detection_image_msg.header = msg.header
            self.detection_image_pub.publish(detection_image_msg)
            
            # Show preview if enabled
            if self.show_preview:
                cv2.imshow('YOLO Detection', annotated_frame)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def draw_3d_box(self, image, detection):
        """Draw 3D bounding box on image using projection"""
        if 'position_3d' not in detection or 'dimensions' not in detection or self.camera_matrix is None:
            return

        # Get 3D parameters
        pos = detection['position_3d']
        dims = detection['dimensions']
        
        cx, cy, cz = pos['x'], pos['y'], pos['z']
        w, h, d = dims['width_m'], dims['height_m'], dims['depth_m']
        
        # Define 8 corners of the box (centered at position)
        # X: right, Y: down, Z: forward
        x_corners = [w/2, w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2]
        y_corners = [h/2, -h/2, -h/2, h/2, h/2, -h/2, -h/2, h/2]
        z_corners = [d/2, d/2, d/2, d/2, -d/2, -d/2, -d/2, -d/2]
        
        # Project corners to 2D
        corners_2d = []
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx_img = self.camera_matrix[0, 2]
        cy_img = self.camera_matrix[1, 2]
        
        for i in range(8):
            # Corner position in camera coordinates
            x = cx + x_corners[i]
            y = cy + y_corners[i]
            z = cz + z_corners[i]
            
            # Project to image plane
            if z <= 0: continue # Skip points behind camera
            
            u = int((x * fx / z) + cx_img)
            v = int((y * fy / z) + cy_img)
            corners_2d.append((u, v))
            
        # Draw lines if we have all corners
        if len(corners_2d) == 8:
            # Define connections
            # Front face (0-3)
            for i in range(4):
                cv2.line(image, corners_2d[i], corners_2d[(i+1)%4], (0, 255, 0), 2)
            
            # Back face (4-7)
            for i in range(4):
                cv2.line(image, corners_2d[4+i], corners_2d[4+((i+1)%4)], (0, 255, 0), 2)
                
            # Connecting lines
            for i in range(4):
                cv2.line(image, corners_2d[i], corners_2d[i+4], (0, 255, 0), 2)
    
    def create_marker(self, detection, marker_id, frame_id):
        """Create a visualization marker for a detection"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "yolo_3d"
        marker.id = marker_id
        marker.action = Marker.ADD
        
        # Determine shape based on class name
        shape_idx = self.get_shape_for_class(detection['class'])
        if shape_idx == 0:
            marker.type = Marker.CUBE
        elif shape_idx == 1:
            marker.type = Marker.SPHERE
        else:
            marker.type = Marker.CYLINDER
        
        # Position
        pos = detection['position_3d']
        marker.pose.position.x = pos['x']
        marker.pose.position.y = pos['y']
        marker.pose.position.z = pos['z']
        
        # Orientation
        if 'obb' in detection:
            # Use OBB rotation
            quat = self.rotation_to_quaternion(detection['obb']['rotation'])
            marker.pose.orientation.x = float(quat[0])
            marker.pose.orientation.y = float(quat[1])
            marker.pose.orientation.z = float(quat[2])
            marker.pose.orientation.w = float(quat[3])
        else:
            # Regular box - no rotation
            marker.pose.orientation.w = 1.0
            
        # Scale - use calculated dimensions
        if 'dimensions' in detection:
            scale_x = detection['dimensions']['height_m']  # Length (X-axis)
            scale_y = detection['dimensions']['width_m']   # Width (Y-axis)
            scale_z = detection['dimensions']['depth_m']   # Depth/Thickness (Z-axis)
        else:
            # Fallback if dimensions not calculated
            scale_x = 0.05
            scale_y = 0.05
            scale_z = 0.03

        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z

        # Color (based on class ID hash or fixed)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.6
        
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        
        return marker

    def create_text_marker(self, detection, marker_id, frame_id):
        """Create a text label marker"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "yolo_text"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        pos = detection['position_3d']
        marker.pose.position.x = pos['x']
        marker.pose.position.y = pos['y']
        marker.pose.position.z = pos['z'] - 0.1  # Slightly above/below
        
        marker.text = f"{detection['class']} ({detection['confidence']:.2f})"
        if 'dimensions' in detection:
            w = detection['dimensions']['width_m'] * 100  # cm
            h = detection['dimensions']['height_m'] * 100 # cm
            d = detection['dimensions']['depth_m'] * 100 # cm (Object thickness/depth)
            marker.text += f"\nW:{w:.1f}cm H:{h:.1f}cm D:{d:.1f}cm"
            
        marker.scale.z = 0.05  # Text height
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        
        return marker
    
    def get_shape_for_class(self, class_name):
        """Determine marker shape based on class name hash"""
        # Sum of ascii values to be deterministic
        val = sum(ord(c) for c in class_name)
        # 0: Cube, 1: Sphere, 2: Cylinder
        return val % 3

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = YoloDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node and node.show_preview:
            cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
