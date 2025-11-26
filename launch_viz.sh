#!/bin/bash

# Source main ROS2 installation first
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source the workspace
source install/setup.bash

echo "Launching myCobot 320 M5 Visualization (RViz2)..."
ros2 launch mycobot_320 test.launch.py
