from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotic_control',  
            executable='go1_jacobian_node',
            name='go1_jacobian_node',
            output='screen',
        )
    ])
