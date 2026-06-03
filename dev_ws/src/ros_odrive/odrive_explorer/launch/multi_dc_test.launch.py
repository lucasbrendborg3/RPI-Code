import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui", default_value="false", description="Start RViz and Joint State Publisher gui."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulation clock if true"
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # --- File Paths ---
    pkg_share = FindPackageShare("odrive_explorer")
    urdf_file = PathJoinSubstitution(
        [pkg_share, "description", "urdf", "multi_dc.urdf.xacro"]
    )
    robot_controllers = PathJoinSubstitution(
        [pkg_share, "config", "multi_dc_controllers.yaml"]
    )
    rviz_config_file = PathJoinSubstitution(
        [pkg_share, "config", "view_robot.rviz"]
    )

    # --- Load URDF ---
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        urdf_file,
    ])
    robot_description = {"robot_description": robot_description_content}

    # --- Nodes ---
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Delay the trajectory controller until joint_state_broadcaster is loaded
    delay_trajectory_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )   

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )

    # Event Handlers for GUI startup order
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(gui)
    )

    delay_joint_state_publisher_gui_after_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=[joint_state_publisher_gui_node],
        ),
        condition=IfCondition(gui)
    )

    nodes_to_start = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_trajectory_controller_after_jsb,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_publisher_gui_after_rviz,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)