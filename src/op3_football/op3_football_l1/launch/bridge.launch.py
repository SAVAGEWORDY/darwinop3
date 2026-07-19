from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='op3_football_l1',
            executable='bridge',
            name='op3_football_bridge',
            output='screen',
        ),
    ])
