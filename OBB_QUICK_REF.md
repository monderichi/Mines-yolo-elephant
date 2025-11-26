# Quick Reference - OBB Pick and Place

## Build Package

```bash
cd /media/monders/Files/robotics/mines/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select yolo_realsense
source install/setup.bash
```

## Download Pre-trained OBB Models

```bash
# Run the download script
./download_obb_models.sh

# Models will be in: ~/.cache/yolo_models/
```

## Launch OBB Detection

```bash
# Basic OBB mode
ros2 launch yolo_realsense yolo_realsense.launch.py \
    model:=yolo11n-obb.pt \
    use_obb:=True

# With custom model
ros2 launch yolo_realsense yolo_realsense.launch.py \
    model:=$HOME/.cache/yolo_models/yolo11n-obb.pt \
    use_obb:=True \
    confidence:=0.7 \
    show_preview:=True
```

## View OBB Detections

```bash
# Structured message with 3D pose
ros2 topic echo /yolo/obb_detections

# JSON format (easier to read)
ros2 topic echo /yolo/detections

# Annotated image
ros2 run rqt_image_view rqt_image_view /yolo/detection_image

# All detection topics
ros2 topic list | grep yolo
```

## Message Structure

```python
# OBBDetection contains:
header              # timestamp and frame_id
class_name          # detected mineral type
confidence          # 0.0 to 1.0

# 2D image space
center_x, center_y  # pixels
width, height       # pixels
rotation            # radians

# 3D camera space
position            # Point(x, y, z) in meters
orientation         # Quaternion(x, y, z, w)
depth               # meters
has_depth           # bool
```

## Train Custom OBB Model

```python
# train_mineral_obb.py
from ultralytics import YOLO

# Load base model
model = YOLO('yolo11n-obb.pt')

# Train on mineral dataset
model.train(
    data='mineral-obb/data.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    name='mineral-obb',
    device='0'  # GPU
)

# Use trained model
# runs/obb/mineral-obb/weights/best.pt
```

## Integration Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from yolo_realsense.msg import OBBDetectionArray

class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place')
        self.sub = self.create_subscription(
            OBBDetectionArray,
            '/yolo/obb_detections',
            self.callback,
            10
        )
    
    def callback(self, msg):
        for det in msg.detections:
            if det.has_depth and det.confidence > 0.7:
                print(f"{det.class_name}: ({det.position.x:.2f}, "
                      f"{det.position.y:.2f}, {det.position.z:.2f})")
                # Send to robot controller

def main():
    rclpy.init()
    node = PickPlaceNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

## Troubleshooting

### No OBB detections
- Verify model has `-obb` suffix
- Check `use_obb:=True` parameter
- Ensure model is trained for OBB

### No depth data
```bash
# Check depth topic
ros2 topic hz /camera/camera/depth/image_rect_raw

# Verify alignment enabled
ros2 param get /camera/camera align_depth.enable
```

### Import errors after build
```bash
# Source workspace
source install/setup.bash

# Verify messages built
ros2 interface show yolo_realsense/msg/OBBDetection
```

## Available Models

**Pre-trained OBB (80 COCO classes)**
- yolo11n-obb.pt (fastest)
- yolo11s-obb.pt
- yolo11m-obb.pt
- yolo11l-obb.pt (most accurate)

**Custom mineral OBB (train your own)**
- See OBB_GUIDE.md for training instructions

## Performance Tips

1. Use `yolo11n-obb.pt` for real-time (30+ FPS)
2. Use GPU if available (`device='0'` in training)
3. Lower resolution for speed: `imgsz=416` or `320`
4. Adjust confidence threshold based on application
5. Filter detections by depth range for valid workspace

## Documentation

- **Full Guide**: [OBB_GUIDE.md](OBB_GUIDE.md)
- **Quick Start**: [QUICK_START.md](QUICK_START.md)
- **Package README**: [src/yolo_realsense/README.md](src/yolo_realsense/README.md)
