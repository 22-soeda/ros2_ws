# -*- coding: utf-8 -*-
"""opponent_detector を（必要なら RealSense ごと）立ち上げる。

    # 検出器だけ（カメラは別で上げてある）
    ros2 launch roboone_perception opponent_detector.launch.py

    # カメラごと。ロボット無しで動作確認できる形（ros-architecture §5 の 4）
    ros2 launch roboone_perception opponent_detector.launch.py camera:=true

    # 俯瞰デバッグ図を見る（別端末で）
    ros2 run rqt_image_view rqt_image_view /detector/debug

**点群は既定で OFF にしてある。** 検出器は深度画像を直接逆投影する（理由は
opponent_detector_node.py の冒頭「ros-architecture から変えたところ」1 項）。
点群は rviz で見たいときだけ pointcloud:=true で足す。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('roboone_perception')
    default_config = os.path.join(share, 'config', 'opponent_detector.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera', default_value='false',
            description='RealSense も一緒に上げるか。既に上がっているなら false'),
        DeclareLaunchArgument(
            'pointcloud', default_value='false',
            description='点群も出すか。検出器は使わない（rviz 用）'),
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='opponent_detector のパラメータ YAML'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='[DEBUG|INFO|WARN|ERROR|FATAL]'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('realsense_bringup'),
                             'launch', 'realsense.launch.py')),
            condition=IfCondition(LaunchConfiguration('camera')),
            launch_arguments={
                'enable_imu': 'true',
                'enable_pointcloud': LaunchConfiguration('pointcloud'),
            }.items()),

        Node(package='roboone_perception', executable='opponent_detector',
             name='opponent_detector', output='screen', emulate_tty=True,
             parameters=[LaunchConfiguration('config')],
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')]),
    ])
