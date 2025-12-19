# myCobot 320 M5 MoveIt2 for ROS 2 Jazzy

Complete MoveIt2 configuration and driver for the **Elephant Robotics myCobot 320 M5** robot arm on **ROS 2 Jazzy** (Ubuntu 24.04).

![myCobot 320 M5](https://www.elephantrobotics.com/wp-content/uploads/2021/03/mycobot-320-m5-1.png)

## 🎯 Features

- ✅ Full MoveIt2 motion planning support
- ✅ Real robot hardware driver using pymycobot
- ✅ RViz visualization with interactive markers
- ✅ Joint state publishing at 20Hz
- ✅ Trajectory execution via FollowJointTrajectory action
- ✅ Simulation mode with joint sliders

## 📋 Prerequisites

### System Requirements
- **OS:** Ubuntu 24.04 LTS
- **ROS 2:** Jazzy Jalisco
- **Robot:** Elephant Robotics myCobot 320 M5

### Install ROS 2 Jazzy
```bash
# Follow official ROS 2 Jazzy installation
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

# Install MoveIt2
sudo apt install ros-jazzy-moveit ros-jazzy-moveit-resources
```

### Install pymycobot
```bash
pip3 install pymycobot
```

## 🔧 Installation

### 1. Create Workspace
```bash
mkdir -p ~/mycobot_ws/src
cd ~/mycobot_ws/src
```

### 2. Clone Repository
```bash
git clone https://github.com/monderichi/Elephant_320m5_moveit2.git
```

### 3. Install Dependencies
```bash
cd ~/mycobot_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build Workspace
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

### 5. Source Workspace
```bash
source ~/mycobot_ws/install/setup.bash
```

> **Tip:** Add this to your `~/.bashrc` for convenience:
> ```bash
> echo "source ~/mycobot_ws/install/setup.bash" >> ~/.bashrc
> ```

## 🚀 Usage

### Simulation Mode (No Robot Required)
Test MoveIt2 without the physical robot:
```bash
ros2 launch mycobot_moveit_config moveit_with_sliders.launch.py
```
- Use the **joint sliders** to move the robot model
- Use the **MotionPlanning** panel to plan and visualize trajectories

### Real Robot Mode
Connect and control the physical myCobot 320 M5:

#### Step 1: Connect Robot via USB
Plug in the myCobot 320 M5 via USB. It should appear as `/dev/ttyACM0` or `/dev/ttyUSB0`.

#### Step 2: Fix USB Serial Permission (Ubuntu 24.04)
```bash
# Create udev rule to prevent ModemManager interference
echo 'ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ENV{BRLTTY_BRAILLE_DEVICE}="no", ENV{ID_MM_DEVICE_IGNORE}="1"' | sudo tee /etc/udev/rules.d/99-mycobot.rules
sudo udevadm control --reload-rules

# Unplug and replug the robot
```

#### Step 3: Launch Real Robot
```bash
ros2 launch mycobot_moveit_config real_robot.launch.py port:=/dev/ttyACM0
```

> **Note:** Change `/dev/ttyACM0` to your actual port (check with `ls /dev/tty*`)

## 📁 Package Structure

```
Elephant_320m5_moveit2/
├── mycobot_description/      # URDF and meshes
│   ├── urdf/                 # Robot description files
│   └── meshes/               # 3D mesh files
├── mycobot_moveit_config/    # MoveIt2 configuration
│   ├── config/               # YAML configuration files
│   ├── launch/               # Launch files
│   └── rviz/                 # RViz configurations
└── mycobot_bringup/          # Robot driver and launch
    └── scripts/              # Python driver scripts
```

## 🎮 Using RViz MotionPlanning Panel

1. **Select Planning Group:** Choose "arm" from the dropdown
2. **Set Goal State:** 
   - Drag the **orange interactive marker** to desired position
   - Or use **Joints** tab to set joint angles
3. **Plan:** Click **Plan** to compute trajectory
4. **Execute:** Click **Execute** to send to robot

## 🔧 Troubleshooting

### "NO PLANNING LIBRARY LOADED" in RViz
This is a cosmetic issue in ROS 2 Jazzy related to parameter types. **Planning still works!** Click "Plan" anyway.

### Robot Not Moving
1. Check USB connection: `ls /dev/ttyACM*`
2. Check driver logs for errors
3. Ensure robot is powered on

### "Permission denied" on Serial Port
```bash
sudo chmod 666 /dev/ttyACM0
# Or add yourself to dialout group:
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Joint States Not Publishing
Check that the driver is running:
```bash
ros2 topic echo /joint_states
```

## 📝 Configuration Files

### Key Files to Modify

| File | Purpose |
|------|---------|
| `config/kinematics.yaml` | IK solver settings |
| `config/joint_limits.yaml` | Joint velocity/acceleration limits |
| `config/moveit_controllers.yaml` | Controller configuration |
| `scripts/mycobot_driver.py` | Robot hardware driver |

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests.

## 📄 License

This project is open source. See individual package licenses.

## 🙏 Acknowledgments

- [Elephant Robotics](https://www.elephantrobotics.com/) for myCobot 320 M5
- [MoveIt2](https://moveit.ros.org/) for motion planning
- [ROS 2](https://www.ros.org/) community

---

**Made with ❤️ for the ROS 2 community**
