# YOLO RealSense - YOLOv11 Object Detection with Intel RealSense D455

This ROS2 package integrates YOLOv11 object detection with the Intel RealSense D455 depth camera.

## Features

- Real-time object detection using YOLOv11
- **OBB (Oriented Bounding Box) support for robotic pick and place**
- Integration with Intel RealSense D455 camera
- **3D position and orientation extraction using depth data**
- Publishes detection results and annotated images
- Configurable YOLO models (nano, small, medium, large, xlarge)
- Adjustable confidence and IOU thresholds

## Prerequisites

- ROS2 Humble
- Intel RealSense SDK 2.0
- Python packages: ultralytics, opencv-python

## Installation

The dependencies should already be installed if you followed the setup instructions.

## Building the Package

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select yolo_realsense
source install/setup.bash
```

## Usage

### Launch with default settings (YOLO11n model)

```bash
ros2 launch yolo_realsense yolo_realsense.launch.py
```

### Launch with custom model

```bash
ros2 launch yolo_realsense yolo_realsense.launch.py model:=yolo11s.pt
```

### Launch with preview window

```bash
ros2 launch yolo_realsense yolo_realsense.launch.py show_preview:=True
```

### Launch with OBB mode for pick and place

```bash
# Using an OBB-trained model
ros2 launch yolo_realsense yolo_realsense.launch.py \
    model:=yolo11n-obb.pt \
    use_obb:=True \
    confidence:=0.6
```

### Launch with custom confidence threshold

```bash
ros2 launch yolo_realsense yolo_realsense.launch.py confidence:=0.7
```

## Available YOLO11 Models

### Regular Detection Models
- `yolo11n.pt` - Nano (fastest, least accurate)
- `yolo11s.pt` - Small
- `yolo11m.pt` - Medium
- `yolo11l.pt` - Large
- `yolo11x.pt` - XLarge (slowest, most accurate)

### OBB (Oriented Bounding Box) Models
- `yolo11n-obb.pt` - Nano OBB
- `yolo11s-obb.pt` - Small OBB
- `yolo11m-obb.pt` - Medium OBB
- `yolo11l-obb.pt` - Large OBB
- `yolo11x-obb.pt` - XLarge OBB

The models will be automatically downloaded on first use.

**For robotic pick and place, use OBB models with `use_obb:=True`**

## Published Topics

### Standard Topics
- `/yolo/detection_image` (sensor_msgs/Image): Annotated image with bounding boxes
- `/yolo/detections` (std_msgs/String): JSON string containing detection information

### OBB Topics (when `use_obb=True`)
- `/yolo/obb_detections` (yolo_realsense/OBBDetectionArray): Oriented detections with 3D pose

## Subscribed Topics

- `/camera/camera/color/image_raw` (sensor_msgs/Image): RGB image from RealSense camera
- `/camera/camera/depth/image_rect_raw` (sensor_msgs/Image): Depth image for 3D coordinates
- `/camera/camera/color/camera_info` (sensor_msgs/CameraInfo): Camera calibration data

## Parameters

- `model_path` (string, default: "yolo11n.pt"): Path to YOLO model
- `confidence_threshold` (double, default: 0.5): Minimum confidence for detections
- `iou_threshold` (double, default: 0.45): IOU threshold for NMS
- `show_preview` (bool, default: False): Show detection preview window
- `use_obb` (bool, default: False): Enable OBB mode for oriented detections

## OBB Mode for Robotic Pick and Place

See **[OBB_GUIDE.md](../../OBB_GUIDE.md)** for complete documentation on:
- Training OBB models
- Using oriented detections
- Integration with robotic arms
- 3D pose extraction
- MoveIt2 integration examples

## Testing the Camera

To test if your RealSense camera is working:

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Launch just the camera
ros2 launch realsense2_camera rs_launch.py

# In another terminal, view the image
ros2 run rqt_image_view rqt_image_view
```

## Viewing Detections

To view the detection results:

```bash
# View annotated images
ros2 run rqt_image_view rqt_image_view /yolo/detection_image

# Echo detection JSON
ros2 topic echo /yolo/detections
```

## Example Detection Output

```json
[
  {
    "class": "person",
    "confidence": 0.89,
    "bbox": {
      "x1": 120.5,
      "y1": 50.2,
      "x2": 300.8,
      "y2": 450.1
    }
  },
  {
    "class": "laptop",
    "confidence": 0.75,
    "bbox": {
      "x1": 350.0,
      "y1": 200.0,
      "x2": 550.0,
      "y2": 400.0
    }
  }
]
```

## Troubleshooting

### Camera not detected

```bash
# Check if camera is connected
rs-enumerate-devices

# Test camera with viewer
realsense-viewer
```

### Permission issues

```bash
# Add user to video group
sudo usermod -aG video $USER
# Log out and log back in
```

### Model download issues

The first time you run a YOLO model, it will be downloaded automatically. Make sure you have internet connection.

## License

MIT
