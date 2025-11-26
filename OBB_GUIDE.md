# OBB (Oriented Bounding Box) Guide for Robotic Pick and Place

## Overview

This guide explains how to use YOLOv11 OBB (Oriented Bounding Boxes) with your RealSense D455 camera for robotic arm pick and place operations. OBB detection provides object orientation in addition to position, which is crucial for accurate grasping.

## What is OBB?

Unlike regular bounding boxes (axis-aligned rectangles), Oriented Bounding Boxes can rotate to match the object's orientation. This provides:

- **Position**: 3D coordinates (x, y, z) in camera frame
- **Orientation**: Rotation angle around Z-axis (yaw) as quaternion
- **Dimensions**: Object width and height in pixels
- **Confidence**: Detection confidence score

This information is essential for robotic arms to:
1. Approach objects from the correct angle
2. Align gripper with object orientation
3. Execute successful pick operations

## Training an OBB Model

### Step 1: Prepare Your Dataset

You need to annotate your mineral dataset with oriented bounding boxes. Use one of these tools:

**Option A: Roboflow (Recommended)**
```bash
# Your existing dataset can be converted to OBB format
# 1. Go to Roboflow: https://roboflow.com
# 2. Upload your images
# 3. Choose "Object Detection (OBB)" as annotation type
# 4. Annotate with rotated boxes
# 5. Export in YOLOv11 OBB format
```

**Option B: LabelImg-OBB or CVAT**
- LabelImg with OBB support: https://github.com/zxl8/LabelImg-OBB
- CVAT with rotated boxes: https://www.cvat.ai/

### Step 2: Dataset Structure

Your OBB dataset should look like this:
```
mineral-obb/
├── data.yaml
├── train/
│   ├── images/
│   └── labels/  # OBB format: class center_x center_y width height rotation
├── valid/
│   ├── images/
│   └── labels/
└── test/
    ├── images/
    └── labels/
```

### Step 3: Train OBB Model

Create a training script `train_obb_model.py`:

```python
from ultralytics import YOLO

# Load a pretrained YOLOv11 OBB model
model = YOLO('yolo11n-obb.pt')  # nano model for speed
# Or: yolo11s-obb.pt, yolo11m-obb.pt, yolo11l-obb.pt, yolo11x-obb.pt

# Train the model
results = model.train(
    data='mineral-obb/data.yaml',  # path to your OBB dataset
    epochs=100,
    imgsz=640,
    batch=16,
    name='mineral-obb-detector',
    device='0',  # Use GPU 0, or 'cpu' for CPU training
    patience=50,  # Early stopping patience
    save=True,
    plots=True
)

# Validate the model
metrics = model.val()

# Export the model
model.export(format='onnx')  # Optional: export for optimization
```

Run training:
```bash
cd /media/monders/Files/robotics/mines/ros2_ws
python3 train_obb_model.py
```

The trained model will be saved in: `runs/obb/mineral-obb-detector/weights/best.pt`

### Step 4: Download Pre-trained OBB Model

If you want to start with a general OBB model:

```bash
# Download YOLOv11 OBB models
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolo11n-obb.pt
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolo11s-obb.pt
```

## Using OBB Mode

### Launch with OBB Detection

```bash
cd /media/monders/Files/robotics/mines/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch with OBB mode enabled
ros2 launch yolo_realsense yolo_realsense.launch.py \
    model:=runs/obb/mineral-obb-detector/weights/best.pt \
    use_obb:=True \
    confidence:=0.6 \
    show_preview:=True
```

### Published Topics

When OBB mode is enabled, the node publishes:

1. **`/yolo/obb_detections`** (yolo_realsense/OBBDetectionArray)
   - Array of oriented detections with full 6-DOF pose information
   - Includes 3D position, orientation quaternion, and dimensions

2. **`/yolo/detections`** (std_msgs/String)
   - JSON format detection data with OBB parameters

3. **`/yolo/detection_image`** (sensor_msgs/Image)
   - Annotated image with oriented bounding boxes drawn

### Viewing OBB Detections

```bash
# View the structured OBB messages
ros2 topic echo /yolo/obb_detections

# View JSON format (easier to read)
ros2 topic echo /yolo/detections

# View annotated images
ros2 run rqt_image_view rqt_image_view /yolo/detection_image
```

## Integration with Robotic Arm

### Example: MoveIt2 Pick and Place

Create a subscriber to process OBB detections:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from yolo_realsense.msg import OBBDetectionArray
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        
        # Subscribe to OBB detections
        self.detection_sub = self.create_subscription(
            OBBDetectionArray,
            '/yolo/obb_detections',
            self.detection_callback,
            10
        )
        
        # TF buffer for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Pick and Place Node initialized')
    
    def detection_callback(self, msg):
        """Process OBB detections for pick and place"""
        if not msg.detections:
            return
        
        # Get the highest confidence detection
        detection = max(msg.detections, key=lambda d: d.confidence)
        
        self.get_logger().info(
            f'Target: {detection.class_name} '
            f'(conf: {detection.confidence:.2f})'
        )
        
        # Create pose for the detected object
        target_pose = PoseStamped()
        target_pose.header = detection.header
        target_pose.pose.position = detection.position
        target_pose.pose.orientation = detection.orientation
        
        # Transform to robot base frame
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # Your robot base frame
                detection.header.frame_id,
                rclpy.time.Time()
            )
            
            target_in_base = do_transform_pose(target_pose, transform)
            
            self.get_logger().info(
                f'Position in base frame: '
                f'x={target_in_base.pose.position.x:.3f}, '
                f'y={target_in_base.pose.position.y:.3f}, '
                f'z={target_in_base.pose.position.z:.3f}'
            )
            
            # Send to MoveIt2 for pick and place
            self.execute_pick_and_place(target_in_base.pose)
            
        except Exception as e:
            self.get_logger().error(f'Transform failed: {str(e)}')
    
    def execute_pick_and_place(self, target_pose):
        """Execute pick and place motion with MoveIt2"""
        # Implement your MoveIt2 motion planning here
        # This is where you would:
        # 1. Plan approach trajectory
        # 2. Open gripper
        # 3. Move to pre-grasp pose
        # 4. Move to grasp pose (using target_pose)
        # 5. Close gripper
        # 6. Lift object
        # 7. Move to place location
        # 8. Open gripper
        pass

def main():
    rclpy.init()
    node = PickAndPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Setting Up TF Frames

You need to publish the transform from camera to robot base:

```python
# Example static transform publisher
# Add to your robot launch file or run separately:

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'camera_link']
            # Adjust x, y, z, roll, pitch, yaw based on your camera mounting
        ),
    ])
```

Or use command line:
```bash
ros2 run tf2_ros static_transform_publisher \
    0 0 0.5 0 0 0 base_link camera_color_optical_frame
```

## OBB Message Structure

The `OBBDetection` message contains:

```
std_msgs/Header header          # Timestamp and frame_id
string class_name               # Detected object class
float32 confidence              # Detection confidence (0-1)

# 2D image space
float32 center_x                # Center X in pixels
float32 center_y                # Center Y in pixels
float32 width                   # Width in pixels
float32 height                  # Height in pixels
float32 rotation                # Rotation in radians

# 3D world space (meters)
geometry_msgs/Point position    # 3D position in camera frame
geometry_msgs/Quaternion orientation  # Orientation as quaternion

# Depth info
float32 depth                   # Distance in meters
bool has_depth                  # Whether depth is valid
```

## Tips for Successful Pick and Place

1. **Calibrate Camera-Robot Transform**
   - Use hand-eye calibration tools
   - Ensure accurate TF tree

2. **Depth Filtering**
   - Filter out detections with `has_depth == False`
   - Set reasonable depth ranges (e.g., 0.3-2.0 meters)

3. **Confidence Threshold**
   - Use higher confidence (0.7-0.8) for critical operations
   - Lower for exploration (0.5-0.6)

4. **Gripper Alignment**
   - Use the orientation quaternion to align gripper
   - Add pre-grasp offset for approach trajectory

5. **Error Handling**
   - Check for obstacles before moving
   - Implement timeout and retry logic
   - Validate grasp success with force sensors

## Troubleshooting

### OBB Model Not Detecting

1. Check if using OBB model:
   ```bash
   # Model filename should contain 'obb'
   yolo11n-obb.pt  # ✓ Correct
   yolo11n.pt      # ✗ Regular detection model
   ```

2. Verify OBB mode is enabled:
   ```bash
   ros2 param get /yolo_detector use_obb
   # Should return: Boolean value is: True
   ```

### No Depth Data

1. Check depth topic:
   ```bash
   ros2 topic echo /camera/camera/depth/image_rect_raw
   ```

2. Verify camera alignment:
   ```bash
   ros2 param get /camera/camera align_depth.enable
   # Should be: True
   ```

### Inaccurate 3D Positions

1. Camera calibration:
   ```bash
   # View camera info
   ros2 topic echo /camera/camera/color/camera_info
   ```

2. Check for:
   - Proper lighting (avoid IR interference)
   - Object distance (0.3-3.0m optimal for D455)
   - Surface reflectivity (avoid highly reflective objects)

## Next Steps

1. **Train Your OBB Model**: Follow the training steps above
2. **Test Detection**: Run with `use_obb:=True`
3. **Integrate with Robot**: Implement pick and place logic
4. **Calibrate**: Fine-tune camera-robot transforms
5. **Deploy**: Test and iterate on real hardware

## Additional Resources

- YOLOv11 OBB Documentation: https://docs.ultralytics.com/tasks/obb/
- RealSense ROS2 Wrapper: https://github.com/IntelRealSense/realsense-ros
- MoveIt2 Tutorials: https://moveit.picknik.ai/humble/index.html
- TF2 Tutorials: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
