# Mines YOLO + Elephant Robotics Integration

ROS2 Humble workspace for mineral detection using YOLO and pick-and-place with myCobot 320 M5.

## Overview

This project combines:
- **Intel RealSense D455** camera for RGB-D imaging
- **YOLOv11** for object detection (with OBB support for oriented bounding boxes)
- **myCobot 320 M5** robotic arm for manipulation
- **ROS2 Humble** as the middleware

## Hardware Requirements

- Intel RealSense D455 (or compatible depth camera)
- Elephant Robotics myCobot 320 M5
- Ubuntu 22.04

## Installation

### 1. Clone the Repository
```bash
git clone https://github.com/monderichi/Mines-yolo-elephant.git
cd Mines-yolo-elephant
```

### 2. Install ROS2 Dependencies
```bash
sudo apt install ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-xacro
```

### 3. Install Python Dependencies
```bash
pip install pymycobot ultralytics pyrealsense2 opencv-python scipy --user
```

### 4. Clone myCobot ROS2 Package
```bash
cd src
git clone -b humble --depth 1 https://github.com/elephantrobotics/mycobot_ros2.git
cd ..
```

### 5. Build the Workspace
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Usage

### Launch YOLO Detection with RealSense
```bash
ros2 launch yolo_realsense yolo_realsense.launch.py
```

### Launch Robot Visualization
```bash
ros2 launch mycobot_320 slider_control.launch.py
```

### Test Robot Connection
```bash
python3 test_connection.py
```

### Diagnose Robot Issues
```bash
python3 diagnose_robot.py
```

## Project Structure

```
ros2_ws/
├── src/
│   ├── yolo_realsense/      # YOLO detection ROS2 package
│   │   ├── msg/             # Custom messages (OBBDetection, etc.)
│   │   ├── launch/          # Launch files
│   │   └── yolo_realsense/  # Python nodes
│   └── mycobot_ros2/        # Robot arm drivers (clone separately)
├── test_connection.py       # Robot connection test
├── diagnose_robot.py        # Robot diagnostic tool
└── README.md
```

## Troubleshooting

### Robot Not Moving
1. Ensure the M5Stack is in **Transponder** mode
2. Check USB permissions: `sudo chmod 666 /dev/ttyACM0`
3. Add user to dialout group: `sudo usermod -a -G dialout $USER`

### Camera Not Found
1. Check USB connection
2. Install librealsense2: `sudo apt install librealsense2-dkms librealsense2-utils`

## License

MIT License

## Credits

- [Elephant Robotics](https://www.elephantrobotics.com/) - myCobot 320
- [Ultralytics](https://ultralytics.com/) - YOLOv11
- [Intel RealSense](https://www.intelrealsense.com/) - D455 Camera
