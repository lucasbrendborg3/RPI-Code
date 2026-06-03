# RPI-Code: ROS2 Robot Arm Control System

A ROS2-based control system for a 5-DOF collaborative robotic arm on Raspberry Pi. Integrates ODrive CAN-based motors and Waveshare UART-based servos through a unified ros2_control interface.

ROS2 Version: Jazzy | Platform: Raspberry Pi (ARM64)

## Overview

The system coordinates:
- 3 ODrive BLDC motors (joints 1-3) via CAN bus at 250 kbps
- 2 Waveshare servos + gripper (joints 4-5, gripper) via USB UART at 1 Mbaud
- Force feedback via load cell (Arduino serial)
- Distance sensing via VL53L1X TOF sensor (I2C)
- RGB-D camera (OAK-D DepthAI)

Control loop rate: 100 Hz using ros2_control framework with separate hardware plugins for each actuator type.

## Hardware

| Component | Model | Interface | Position Range | Notes |
|-----------|-------|-----------|-----------------|-------|
| Joint 1 | ODrive | CAN | ±3.12 rad | 42.13:1 gearbox |
| Joint 2 | ODrive | CAN | ±0.897 to 1.92 rad | 303.5:1 gearbox |
| Joint 3 | ODrive | CAN | ±1.83 rad | 194.3:1 gearbox |
| Joint 4 | Waveshare ST3025 | UART | ±2.09 rad | 1:1 ratio |
| Joint 5 | Waveshare ST3025 | UART | ±2.09 rad | 1:1 ratio |
| Gripper | Waveshare ST3025 | UART | ±0.3 to 0.52 rad | 5.2:1 gearbox |

## Repository Structure

```
RPI-Code/dev_ws/src/
├── arm_control/              # Main hardware interface package
│   ├── launch/
│   │   ├── arm.launch.py             # Complete system
│   │   └── hardware_only.launch.py   # Headless mode
│   ├── config/ros2_controllers_hw.yaml
│   └── description/ros2_control/hardware_ros2_control.xacro
│
├── robotv2_description/      # URDF robot model
│   ├── urdf/robots/robotv2.urdf.xacro
│   └── launch/robot_state_publisher.launch.py
│
├── ros_odrive/               # ODrive integration (C++)
├── ros_waveshare/            # Waveshare integration (C++)
├── load_cell/                # Force sensor node (Python)
├── gripper_controller/       # Gripper control node (Python)
└── tof_sensor/               # Distance sensor node (Python)
```

## Packages

**arm_control** - Main hardware interface that coordinates ODrive and Waveshare actuators. Publishes joint states at 100 Hz and manages all controllers.

**robotv2_description** - URDF-based robot model defining kinematic structure, visual geometry, and joint limits. Enables robot state publishing and TF broadcasting.

**ros_odrive** - Handles ODrive motor controllers via CAN bus. Provides standalone node and ros2_control hardware interface plugin for 3 BLDC motors.

**ros_waveshare** - Manages Waveshare bus servos over UART serial at 1 Mbaud. Handles 2 arm servos plus gripper servo.

**load_cell** - Python node that reads force/weight from HX711 load cell via Arduino serial. Publishes to `/weight` (std_msgs/Float64).

**tof_sensor** - Python node for VL53L1X time-of-flight distance sensor via I2C. Publishes to `/range` (sensor_msgs/Range).

**gripper_controller** - Python node providing high-level gripper control. Translates text commands into servo position goals.

## Installation

### Install ROS2 Jazzy

```bash
sudo apt update
sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros2.org/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-ros-core ros-jazzy-ros-base ros-dev-tools
source /opt/ros/jazzy/setup.bash
```

### Install Dependencies

```bash
sudo apt install python3-pip git cmake build-essential
sudo apt install ros-jazzy-hardware-interface ros-jazzy-controller-manager
sudo apt install ros-jazzy-robot-state-publisher ros-jazzy-joint-state-broadcaster
sudo apt install ros-jazzy-joint-trajectory-controller ros-jazzy-control-msgs

pip install colcon-common-extensions setuptools
pip install pyserial Adafruit-CircuitPython-HX711 Adafruit-CircuitPython-VL53L1X
pip install python-can RPi.GPIO
```

### Setup Hardware

```bash
# CAN interface
sudo modprobe can can_raw mcp251x
sudo ip link add dev can0 type can bitrate 250000
sudo ip link set can0 up

# USB serial permissions
sudo usermod -a -G dialout $USER
newgrp dialout

# GPIO access
python3 -c "import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BCM); GPIO.setup(22, GPIO.IN); print('GPIO ready')"
```

### Clone and Build

```bash
cd ~
git clone https://github.com/lucasbrendborg3/RPI-Code.git
cd RPI-Code/dev_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --parallel-workers 2
source install/setup.bash
```

## Building & Running

### Build

```bash
cd ~/RPI-Code/dev_ws
colcon build --parallel-workers 2
source install/setup.bash

# Selective build
colcon build --packages-select arm_control robotv2_description

# Clean rebuild
rm -rf build/ install/ log/
colcon build
```

### Running the System

Launch the complete system:
```bash
ros2 launch arm_control arm.launch.py
```

This starts:
- Hardware interface (ODrive + Waveshare)
- Joint state broadcaster
- Robot state publisher with TF
- Load cell sensor node
- TOF distance sensor node
- Gripper controller node


## Monitoring

```bash
# Monitor joint states
ros2 topic echo /joint_states

# Check active nodes
ros2 node list

# Check published topics
ros2 topic list -v

# Check controller status
ros2 service call /controller_manager/list_controllers ros2controlcmds/srv/ListControllers
```

## Sending Commands

Move ODrive motor (Joint 1):
```bash
ros2 topic pub /arm_controller/commands std_msgs/Float64MultiArray "data: [0.5, 0.0, 0.0, 0.0, 0.0]"
```

Control gripper:
```bash
ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 2.0}}"
```

Test individual nodes:
```bash
# Load cell
ros2 run load_cell load_cell_node
ros2 topic echo /weight

# TOF sensor
ros2 run tof_sensor tof_sensor_node
ros2 topic echo /range

# Gripper
ros2 run gripper_controller gripper_translator_node
```

## Configuration

Edit `arm_control/config/ros2_controllers_hw.yaml` to change:
- `update_rate` - Control loop frequency (default: 100 Hz)
- Joint names and controller types
- Joint state broadcaster settings
