# Quick Start Guide - YOLOv11 with Intel RealSense D455 on ROS2

## Complete Setup Summary

You now have a fully configured system with:
- ✅ ROS2 Humble
- ✅ Intel RealSense SDK and drivers
- ✅ YOLOv11 (Ultralytics) with OBB support
- ✅ ROS2 RealSense wrapper
- ✅ Custom `yolo_realsense` package with 6-DOF pose estimation
- ✅ Web-based viewer for headless systems
- ✅ Custom OBB messages for robotic pick and place

## Quick Commands

### 1. Test Your RealSense Camera

```bash
# Check if camera is detected
rs-enumerate-devices

# Launch RealSense viewer (GUI)
realsense-viewer
```

### 2. Run YOLO Detection with RealSense

```bash
# Navigate to workspace
cd /media/monders/Files/robotics/mines/ros2_ws

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch with regular detection
ros2 launch yolo_realsense yolo_realsense.launch.py

# Launch with OBB (Oriented Bounding Box) for pick and place
ros2 launch yolo_realsense yolo_realsense.launch.py \
  model:=/home/monders/.cache/yolo_models/yolo11n-obb.pt \
  use_obb:=True \
  confidence:=0.5
```

**Note:** Pre-trained OBB models detect COCO objects (vehicles, animals, etc.), not minerals. For mineral detection, train a custom model (see OBB_TRAINING_GUIDE.md).

## Viewing Results

### ROS2 Topics

Monitor detections:
```bash
# View regular detections
# Terminal 3: Echo detection data (JSON)
ros2 topic echo /yolo/detections

# Terminal 4: View 3D Markers in RViz
rviz2
# In RViz:
# 1. Set Fixed Frame to 'camera_link' or 'camera_color_optical_frame'
# 2. Add -> MarkerArray -> Topic: /yolo/markers
# 3. Add -> Image -> Topic: /yolo/detection_image
# 4. Add -> PointCloud2 -> Topic: /camera/camera/depth/color/points (if available)
```
### 4. Advanced Usage

# View OBB detections (with 3D pose)
ros2 topic echo /yolo/obb_detections

# View detection image
ros2 topic echo /yolo/detection_image
```

### Web Viewer (Recommended for Headless Systems)

If you're on a headless system (SSH/remote) without GUI support:

1. **Start the web viewer:**
```bash
cd ~/ros2_ws
python3 web_viewer.py
```

2. **Open browser:**
Navigate to `http://localhost:8080` to see:
- Live detection stream
- Real-time FPS counter
- Detection table with 3D coordinates (X, Y, Z in meters)
- Class names and confidence scores

**Features:**
- Auto-refresh every 100ms
- Shows depth-integrated 3D positions
- Works over SSH with port forwarding: `ssh -L 8080:localhost:8080 user@robot`

### View Image Stream (GUI Systems)

Use RViz2 or rqt_image_view:
```bash
# Option 1: RViz2
rviz2

# Option 2: rqt_image_view  
ros2 run rqt_image_view rqt_image_view /yolo/detection_image
```

**Note:** OpenCV GUI windows (`cv2.imshow`) are not available on headless systems. Use the web viewer instead.
### 4. Advanced Usage

```bash
# Use different YOLO model (more accurate but slower)
ros2 launch yolo_realsense yolo_realsense.launch.py model:=yolo11m.pt

# Adjust confidence threshold
ros2 launch yolo_realsense yolo_realsense.launch.py confidence:=0.7

# Enable preview window (only works with GUI, not headless)
ros2 launch yolo_realsense yolo_realsense.launch.py show_preview:=True

# Use custom trained mineral model (regular detection)
ros2 launch yolo_realsense yolo_realsense.launch.py model:=runs/detect/train/weights/best.pt

# Use custom trained mineral model (OBB mode)
ros2 launch yolo_realsense yolo_realsense.launch.py \
  model:=runs/obb/mineral-obb-detector/weights/best.pt \
  use_obb:=True \
  confidence:=0.5
```

## Available YOLO Models

### Regular Detection
From fastest to most accurate:
- `yolo11n.pt` - Nano (fastest, ~1-2ms)
- `yolo11s.pt` - Small
- `yolo11m.pt` - Medium
- `yolo11l.pt` - Large
- `yolo11x.pt` - Extra Large (most accurate, slowest)

### OBB (Oriented Bounding Box) for Pick and Place
- `yolo11n-obb.pt` - Nano OBB
- `yolo11s-obb.pt` - Small OBB  
- `yolo11m-obb.pt` - Medium OBB
- `yolo11l-obb.pt` - Large OBB
- `yolo11x-obb.pt` - Extra Large OBB

**Use OBB models with `use_obb:=True` for robotic arm applications!**

## Environment Setup (add to ~/.bashrc)

Add these lines to your `~/.bashrc` for convenience:

```bash
# ROS2 Humble
source /opt/ros/humble/setup.bash

# Your workspace
source /media/monders/Files/robotics/mines/ros2_ws/install/setup.bash

# Python local bin (for ultralytics/yolo commands)
export PATH="$HOME/.local/bin:$PATH"
```

Then reload:
```bash
source ~/.bashrc
```

## Useful ROS2 Commands

```bash
# List all active topics
ros2 topic list

# Show topic info
ros2 topic info /yolo/detection_image

# Show camera parameters
ros2 param list /camera/camera

# Record data (camera + detections)
ros2 bag record /camera/camera/color/image_raw /yolo/detection_image /yolo/detections

# View all nodes
ros2 node list

# Check node info
ros2 node info /yolo_detector
```

## Troubleshooting

### Camera not found

```bash
# Check USB connection
lsusb | grep Intel

# Check if camera is detected
rs-enumerate-devices

# Check camera permissions
groups $USER  # Should include 'video'

# If not, add yourself to video group:
sudo usermod -aG video $USER
# Then log out and back in
```

### OpenCV GUI errors (headless systems)

If you see: `cvShowImage: OpenCV Error: The function is not implemented`

**Solution:** Your system is headless (no GUI). Use the web viewer instead:
```bash
python3 web_viewer.py
# Open browser: http://localhost:8080
```

Or disable preview:
```bash
ros2 launch yolo_realsense yolo_realsense.launch.py show_preview:=False
```

### Depth coordinate errors

If you see "index out of bounds" errors:
- System automatically scales from color (1280x720) to depth (848x480)
- Ensure you're using latest yolo_detector.py
- Check depth stream: `ros2 topic hz /camera/depth/image_rect_raw`

### OBB models detecting wrong objects

Pre-trained OBB models detect COCO dataset (80 general classes), not minerals.

**Solution:** Train custom OBB model on your mineral dataset:
```bash
# See complete guide
cat OBB_TRAINING_GUIDE.md
```

### Rebuild after changes

```bash
cd /media/monders/Files/robotics/mines/ros2_ws
rm -rf build/yolo_realsense install/yolo_realsense
source /opt/ros/humble/setup.bash
colcon build --packages-select yolo_realsense
source install/setup.bash
```

### Check RealSense firmware

```bash
rs-fw-update -l  # List devices
# Update if needed following prompts
```

## What You Can Detect

### Pre-trained Models (COCO Dataset)
YOLO11 can detect 80 object classes:
- People, animals (cat, dog, bird, etc.)
- Vehicles (car, bus, truck, bicycle, motorcycle)
- Household items (chair, couch, bed, table)
- Electronics (laptop, mouse, keyboard, cell phone, TV)
- Kitchen items (bottle, cup, fork, knife, spoon, bowl)

**Limitation:** These do NOT detect minerals. You need custom training.

### Custom Mineral Detection (After Training)
Your dataset includes 15 mineral classes:
- Alunite, Andesite, Azurite, Basalt, Bauxite
- Chalcopyrite, Diorite, Galena, Gold, Limestone
- Malachite, Pyrite, Quartz, Sandstone, Sphalerite

**To train:** See `MINERAL_DETECTION_GUIDE.md` and `OBB_TRAINING_GUIDE.md`

## Next Steps

1. **Test Web Viewer**: Open http://localhost:8080 to see live detections with 3D coordinates
2. **Custom Training**: Train YOLO on your mineral dataset (see guides)
3. **OBB Training**: Annotate minerals with rotated boxes for pick and place
4. **Robotic Integration**: Subscribe to `/yolo/obb_detections` for 6-DOF poses
5. **Fine-tuning**: Adjust confidence thresholds and camera parameters

## Documentation

- `QUICK_START.md` - This file (getting started)
- `MINERAL_DETECTION_GUIDE.md` - Regular detection training
- `OBB_TRAINING_GUIDE.md` - Oriented bounding box training for robotics
- `SESSION_LOG.md` - Complete development session log
5. **Robot Control**: Integrate detections with MoveIt2 or your motion planner

## Support

- RealSense: https://github.com/IntelRealSense/librealsense
- YOLO11: https://docs.ultralytics.com/models/yolo11/
- ROS2: https://docs.ros.org/en/humble/

Happy detecting! 🚀
