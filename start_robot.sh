#!/bin/bash

# Wait a few seconds for the system to settle
sleep 5

echo "Starting robot sequence..."

# Step 1: Run the homing script and wait for it to complete
echo "Executing homing script..."
/usr/bin/python3 /home/arm/RPI-Code/Homing.py
echo "Homing complete."

# Step 2: Source the ROS2 workspace and launch the arm controller
echo "Sourcing ROS2 workspace and launching arm control..."
# source /home/arm/dev_ws/install/setup.bash
# ros2 launch arm_control arm.launch.py

echo "Robot startup sequence finished."
