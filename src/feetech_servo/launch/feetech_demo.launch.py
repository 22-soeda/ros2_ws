"""feetech_demo node を config/feetech_demo.yaml で起動する。

    ros2 launch feetech_servo feetech_demo.launch.py
    # 実機を動かす場合:
    ros2 launch feetech_servo feetech_demo.launch.py enable_motion:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("feetech_servo")
    params = os.path.join(pkg, "config", "feetech_demo.yaml")

    enable_motion = LaunchConfiguration("enable_motion")

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_motion", default_value="false",
            description="true で実機を動かす（トルクON+サイン波）。既定は読み取りのみ。"),
        Node(
            package="feetech_servo",
            executable="feetech_demo_node",
            name="feetech_demo",
            output="screen",
            parameters=[params, {"enable_motion": enable_motion}],
        ),
    ])
