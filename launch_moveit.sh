#!/bin/bash
# Launch MoveIt2 for myCobot 320 M5
#
# NOTE: There is a known bug in MoveIt2 ROS2 Humble where the RViz Motion 
# Planning plugin crashes due to a parameter type mismatch. The move_group
# node still works correctly - you can use Python/C++ APIs or command-line 
# tools for motion planning.
#
# Usage:
#   ./launch_moveit.sh                # Launch with RViz (may crash)
#   ./launch_moveit.sh --no-rviz     # Launch without RViz

cd /media/monders/Files/robotics/mines/ros2_ws
source /opt/ros/humble/setup.bash
source /media/monders/Files/robotics/mines/ros2_ws/install/setup.bash

echo "Launching MoveIt2 for myCobot 320 M5..."
echo "Note: RViz Motion Planning plugin may crash - this is a known ROS2 Humble bug."
echo "The move_group node works correctly via Python/C++ API."
echo ""

ros2 launch mycobot_320_moveit2 demo.launch.py
