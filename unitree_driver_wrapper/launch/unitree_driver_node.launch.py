from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unitree_ros',
            executable='unitree_driver_node',
            name='unitree_driver_node',
            output='screen',
        ),
    ])
