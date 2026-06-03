#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_dir = get_package_share_directory('arm_control')
    
    # URDF file (corrected to match your actual file)
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            os.path.join(pkg_dir, "description", "urdf", "test.urdf.xacro"),
        ]),
        value_type=str
    )
    
    # Controllers configuration (matches your actual config file)
    robot_controllers = os.path.join(pkg_dir, "config", "ros2_controllers_hw.yaml")
    
    # Base nodes for headless operation
    nodes = [
        # ROS2 Control Node (hardware interface)
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_content},
                robot_controllers,
                {'use_sim_time': False}
            ],
            output="screen",
        ),
        
        # Robot State Publisher (publishes TF and robot description)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description_content},
                {'use_sim_time': False}
            ],
        ),
        
        # Joint State Broadcaster (immediate start)
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        
        # Arm Controller (delayed start to ensure hardware initialization)
        TimerAction(
            period=2.5,  # Delay for hardware ready
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["arm_controller", "--controller-manager", "/controller_manager"],
                    output="screen",
                )
            ]
        ),
        
        # Gripper Controller (delayed start after arm controller)
        TimerAction(
            period=4.0,  # Delay after arm controller
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["gripper_action_controller", "--controller-manager", "/controller_manager"],
                    output="screen",
                )
            ]
        ),
    ]
    
    return LaunchDescription(nodes)