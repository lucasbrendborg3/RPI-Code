#!/bin/bash

# Wait a few seconds for the system to settle
sleep 10

echo "Starting robot sequence..."

# Step 1: Run the homing script and wait for it to complete
echo "Executing homing script..."
/usr/bin/python3 /home/arm/RPI-Code/Homing.py
echo "Homing complete."

# Step 2: Source the ROS2 workspace and launch the arm controller
echo "Sourcing ROS2 workspace and launching arm control..."

source /opt/ros/jazzy/setup.bash

source /home/arm/RPI-Code/dev_ws/install/setup.bash

export ROS_DOMAIN_ID=23


source /home/arm/RPI-Code/dev_ws/install/setup.bash

cd /home/arm

ros2 launch arm_control arm.launch.py
# ros2 launch arm_control arm.launch.py > /home/arm/ros2_boot.log 2>&1

echo "Robot startup sequence finished."
