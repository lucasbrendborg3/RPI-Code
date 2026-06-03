from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch three ODrive CAN nodes for motors 0, 1, and 2 on the same CAN bus."""
    
    return LaunchDescription([
        Node(
            package='odrive_can',
            executable='odrive_can_node',
            name='odrive_can_node_0',
            namespace='odrive_axis0',
            parameters=[{
                'node_id': 0,
                'interface': 'can0'  # CAN interface on the Raspberry Pi (DONT CHANGE)
            }]
        ),
        
        Node(
            package='odrive_can',
            executable='odrive_can_node',
            name='odrive_can_node_1',
            namespace='odrive_axis1',
            parameters=[{
                'node_id': 1,
                'interface': 'can0'
            }]
        ),
        
        Node(
            package='odrive_can',
           executable='odrive_can_node',
            name='odrive_can_node_2',
            namespace='odrive_axis2',
            parameters=[{
                'node_id': 2,
                'interface': 'can0'
            }]
        )
    ])