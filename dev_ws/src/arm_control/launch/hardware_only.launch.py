import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Use robotv2_description package (the one you copied)
    robotv2_description_pkg = FindPackageShare('robotv2_description')
    
    # Use the main robotv2.urdf.xacro from robotv2_description
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                robotv2_description_pkg,
                'urdf', 'robots', 'robotv2.urdf.xacro'
            ]),
            ' robot_name:=robotv2',
            ' add_world:=true',
            ' use_gripper:=true',
            ' use_gazebo:=false',  # Important: disable Gazebo for Pi
            ' use_rviz:=false'     # Important: disable RViz visuals
        ]),
        value_type=str
    )
    
    robot_description = {"robot_description": robot_description_content}

    # Use your Pi controller config (from arm_control or create in robotv2_description)
    # Option 1: Use arm_control config
    arm_control_pkg = FindPackageShare('arm_control')
    controller_config = PathJoinSubstitution([
        arm_control_pkg,
        'config', 'ros2_controllers_hw.yaml'
    ])
    
    # Option 2: Use robotv2_description config (if you have pi_controllers.yaml there)
    # controller_config = PathJoinSubstitution([
    #     robotv2_description_pkg,
    #     'config', 'pi_controllers.yaml'
    # ])

    # Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="both",
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner", 
        arguments=["gripper_action_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])