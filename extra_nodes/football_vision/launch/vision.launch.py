"""
Запуск камеры (usb_cam) + ноды распознавания football_vision + HTTP MJPEG стрима.

    ros2 launch football_vision vision.launch.py

Если камера уже поднята другим лаунчем (op3_bringup/strategy), запусти только ноду:
    ros2 launch football_vision vision.launch.py with_camera:=false
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    with_camera = LaunchConfiguration("with_camera")
    with_stream = LaunchConfiguration("with_stream")
    stream_host = LaunchConfiguration("stream_host")
    stream_port = LaunchConfiguration("stream_port")
    config = os.path.join(
        get_package_share_directory("football_vision"), "config", "vision.yaml")

    usb_cam = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_node",
        namespace="usb_cam_node",
        output="log",
        condition=IfCondition(with_camera),
        parameters=[{
            "video_device": "/dev/video0",
            "image_width": 1280,
            "image_height": 720,
            "framerate": 30.0,
            "pixel_format": "mjpeg2rgb",
        }],
        remappings=[("/image_raw", "/usb_cam_node/image_raw")],
    )

    vision = Node(
        package="football_vision",
        executable="vision_node",
        name="football_vision",
        output="screen",
        parameters=[config],
    )

    stream = Node(
        package="football_vision",
        executable="stream_node",
        name="football_vision_stream",
        output="screen",
        condition=IfCondition(with_stream),
        parameters=[{
            "image_topic": "/vision/image_annotated",
            "host": stream_host,
            "port": stream_port,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("with_camera", default_value="true"),
        DeclareLaunchArgument("with_stream", default_value="true"),
        DeclareLaunchArgument("stream_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("stream_port", default_value="8080"),
        usb_cam,
        vision,
        stream,
    ])
