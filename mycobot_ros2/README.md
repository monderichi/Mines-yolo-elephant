# MoveIt 2 Robot Arm Simulation - ROS 2 Jazzy

This branch contains a complete MoveIt 2 setup for simulating the myCobot 280 robot arm with ROS 2 Jazzy.

## 🚀 Quick Start (Docker)

### Prerequisites
- Docker with GPU support (WSL2 with WSLg on Windows, or native Linux)
- ROS 2 Jazzy Docker image

### 1. Start ROS 2 Jazzy Container

```bash
docker run -it --rm \
  --name ros2_moveit \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  osrf/ros:jazzy-desktop \
  bash
```

### 2. Copy Repository to Container

```bash
# From host (outside container)
docker cp mycobot_ros2 ros2_moveit:/root/ros2_ws/src/
```

### 3. Install Dependencies

```bash
# Inside container
cd /root/ros2_ws
apt-get update
rosdep update
rosdep install -i --from-path src --rosdistro jazzy -y
```

### 4. Build Workspace

```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 5. Launch Gazebo Simulation (Terminal 1)

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch mycobot_gazebo mycobot.gazebo.launch.py use_rviz:=false
```

### 6. Launch MoveIt 2 with RViz (Terminal 2)

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch mycobot_moveit_config move_group.launch.py use_sim_time:=true
```

### Alternative: MoveIt Demo Only (No Gazebo)

```bash
ros2 launch mycobot_moveit_config move_group.launch.py use_sim_time:=false
```

## 📦 Packages Included

| Package | Description |
|---------|-------------|
| `mycobot_description` | Robot URDF, meshes, and visual models |
| `mycobot_moveit_config` | MoveIt 2 configuration (OMPL, Pilz planners) |
| `mycobot_gazebo` | Gazebo simulation models and worlds |
| `mycobot_bringup` | Launch scripts for bring-up |
| `mycobot_interfaces` | Custom ROS 2 messages/services |
| `mycobot_moveit_demos` | MoveIt demonstration nodes |
| `mycobot_system_tests` | System test package |
| `mycobot_ros2` | Metapackage |

## 🎮 Using MoveIt in RViz

1. **Planning Group**: Select `arm` in the MotionPlanning panel
2. **Move End-Effector**: Drag the orange interactive marker
3. **Plan**: Click "Plan" to compute trajectory
4. **Execute**: Click "Execute" to animate the motion

## ⚙️ Planners Available

- **OMPL** (default) - Sampling-based motion planning
- **Pilz Industrial Motion Planner** - Industrial motion primitives (PTP, LIN, CIRC)

## 🔧 Troubleshooting

### RViz doesn't show robot
- Ensure `Fixed Frame` is set to `base_link`
- Check that `robot_description` topic is publishing

### move_group crashes
- The STOMP planner has been removed due to Jazzy compatibility issues
- Use OMPL or Pilz planners instead

### GUI not displaying (Docker)
- Ensure X11 forwarding is configured
- On WSL2: WSLg should work automatically
- On Linux: Run `xhost +local:docker` before starting container

## 📚 References

- [Original Tutorial](https://automaticaddison.com/configure-moveit-2-for-a-simulated-robot-arm-ros-2-jazzy/)
- [MoveIt 2 Documentation](https://moveit.picknik.ai/main/index.html)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)

## 📝 Notes

- MTC (MoveIt Task Constructor) demos are excluded due to API changes in Jazzy
- Based on [automaticaddison/mycobot_ros2](https://github.com/automaticaddison/mycobot_ros2) jazzy branch
