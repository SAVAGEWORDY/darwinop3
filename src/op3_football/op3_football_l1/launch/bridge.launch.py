from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('op3_football_l1'),
        'config',
        'auto_getup.yaml',
    )
    return LaunchDescription([
        Node(
            package='op3_football_l1',
            executable='bridge',
            name='op3_football_bridge',
            output='screen',
            parameters=[params_file],
        ),
    ])
