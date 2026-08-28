# -*- coding: utf-8 -*-
"""behavior を（必要なら検出器ごと）立ち上げる。

    # 行動層だけ（検出器と motion は別で上げてある）
    ros2 launch roboone_behavior behavior.launch.py

    # 検出器ごと。カメラも上げるなら camera:=true
    ros2 launch roboone_behavior behavior.launch.py detector:=true camera:=true

    # 技を出さずに歩行だけ見る（間合いに入っても /cmd_motion を出さない）
    ros2 launch roboone_behavior behavior.launch.py techniques:="[]"

**このノードは /autonomy が true になるまで /cmd_walk を出さない。** 上げただけ
では機体は動かない。teleop 側で自律動作のボタンを長押しすること
（roboone_teleop の README）。止めるのも teleop 側の割り込み 4 つのいずれか。
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('roboone_behavior')
    default_config = os.path.join(share, 'config', 'behavior.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'detector', default_value='false',
            description='opponent_detector も一緒に上げるか'),
        DeclareLaunchArgument(
            'camera', default_value='false',
            description='RealSense も上げるか（detector:=true のときだけ効く）'),
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='behavior のパラメータ YAML'),
        DeclareLaunchArgument(
            'techniques', default_value="['punch_r', 'punch_l']",
            description='ENGAGE で出す技。[] にすると技を出さない'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='[DEBUG|INFO|WARN|ERROR|FATAL]'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('roboone_perception'),
                             'launch', 'opponent_detector.launch.py')),
            condition=IfCondition(LaunchConfiguration('detector')),
            launch_arguments={'camera': LaunchConfiguration('camera')}.items()),

        Node(package='roboone_behavior', executable='behavior',
             name='behavior', output='screen', emulate_tty=True,
             parameters=[LaunchConfiguration('config'),
                         # コマンドラインから来るのは文字列なので、
                         # 文字列の配列だと明示しないと型が合わない
                         {'robot.techniques': ParameterValue(
                             LaunchConfiguration('techniques'),
                             value_type=List[str])}],
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')]),
    ])
