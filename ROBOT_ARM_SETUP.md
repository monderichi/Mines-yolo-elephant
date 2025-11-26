# Robot Arm Setup (myCobot 320)

## Installation Status
- **ROS2 Package**: `mycobot_ros2` (Humble branch) - **INSTALLED & BUILT**
- **Python API**: `pymycobot` - **INSTALLED**

## Next Steps

### 1. Source the Workspace
You need to source the workspace to use the new packages.
```bash
source install/setup.bash
```

### 2. USB Permissions
To communicate with the robot arm via USB, you likely need to grant permission to the serial port.
Add your user to the `dialout` group:
```bash
sudo usermod -a -G dialout $USER
```
*Note: You may need to log out and log back in for this to take effect.*

### 3. Identify Your Model
**SELECTED MODEL: myCobot 320 M5** (Uses M5Stack Basic at the base)

### 4. Testing the Installation

**Step A: Verify Serial Connection**
Run the Python script to check if the computer can talk to the robot:
```bash
python3 test_connection.py
```

**Step B: Launch Visualization (URDF)**
This will open RViz and show the 3D model of the robot.
```bash
ros2 launch mycobot_320 test.launch.py
```

**Step C: Slider Control Demo**
This allows you to move the real robot (if connected) or just the simulation using sliders.
```bash
ros2 launch mycobot_320 slider_control.launch.py
```

## Integration Plan
Once the arm is verified, we will proceed to:
1. Create a move script to test hardware control.
2. Integrate with the YOLO detection system.
3. Implement coordinate transformation (Camera -> Robot Base).
4. Execute Pick and Place.
