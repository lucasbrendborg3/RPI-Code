import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart # Added OnProcessStart for flexibility
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.parameter_descriptions import ParameterValue
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
    pkg_share = FindPackageShare("waveshare_explorer")
    # URDF using the single_servo definition
    urdf_file = PathJoinSubstitution(
        [pkg_share, "description", "urdf", "single_servo.urdf.xacro"]
    )
    # Controllers config for the single servo
    robot_controllers = PathJoinSubstitution(
        [pkg_share, "config", "single_servo_controllers.yaml"]
    )
    # RViz config
    rviz_config_file = PathJoinSubstitution(
        [pkg_share, "description/rviz", "view_robot.rviz"] # Assuming this exists
    )

    # --- Load URDF ---
    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", urdf_file]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
        "use_sim_time": use_sim_time
    }

    # --- Nodes ---
    # Controller Manager Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers], # Pass both robot description and controllers path
        output="both",
    )

    # Robot State Publisher Node (Matches example.launch.py name)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description], # Pass robot_description dict
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Forward Position Controller Spawner (Specific to single servo test)
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    # RViz Node (Conditional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # Joint State Publisher GUI Node (Conditional)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )

    # --- Event Handlers for GUI startup order ---
    # Delay RViz start until Joint State Broadcaster spawner finishes
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(gui) # Only activate this handler if gui is true
    )

    # Delay Joint State Publisher GUI start until RViz exits (or starts, adjust if needed)
    delay_joint_state_publisher_gui_after_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=[joint_state_publisher_gui_node],
        ),
        condition=IfCondition(gui) # Only activate this handler if gui is true
    )

    # --- List of items to launch ---
    # Add nodes and spawners directly. Controller Manager should be ready for spawners shortly after it starts.
    nodes_to_start = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,         # Start spawner directly
        forward_position_controller_spawner,    # Start spawner directly
        # GUI related handlers (only added if gui=true due to their internal conditions)
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_publisher_gui_after_rviz,
    ]

    # --- Launch file prints for better understanding of startup ---
    print(f"Nodes: {nodes_to_start}")
    print(f"robot_description: {robot_description}")
    print(f"robot_controllers: {robot_controllers}")

    return LaunchDescription(declared_arguments + nodes_to_start)