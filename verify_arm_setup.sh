#!/bin/bash

# Source the workspace
source install/setup.bash

echo "Verifying myCobot installation..."

# Check if packages are found
if ros2 pkg list | grep -q "mycobot_320"; then
    echo "[OK] mycobot_320 package found."
else
    echo "[ERROR] mycobot_320 package NOT found."
fi

if ros2 pkg list | grep -q "mycobot_320pi"; then
    echo "[OK] mycobot_320pi package found."
else
    echo "[ERROR] mycobot_320pi package NOT found."
fi

# Check for pymycobot
if python3 -c "import pymycobot" 2>/dev/null; then
    echo "[OK] pymycobot Python library found."
else
    echo "[ERROR] pymycobot Python library NOT found."
fi

# Check for rviz2
if which rviz2 > /dev/null; then
    echo "[OK] rviz2 found."
else
    echo "[ERROR] rviz2 NOT found. Please install ros-humble-rviz2."
fi

echo "---------------------------------------------------"
echo "To test the visualization, run one of the following:"
echo "  ros2 launch mycobot_320 test.launch.py   (For M5Stack version)"
echo "  ros2 launch mycobot_320pi test.launch.py (For Pi version)"
echo "---------------------------------------------------"
