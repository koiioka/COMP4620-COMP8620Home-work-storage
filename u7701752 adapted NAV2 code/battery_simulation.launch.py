from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_repair_pkg',
            executable='battery_manager',
            name='battery_manager'
        ),
    ])
