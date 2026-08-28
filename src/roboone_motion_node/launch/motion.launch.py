# -*- coding: utf-8 -*-
"""motion ノードだけを立ち上げる launch。

    ros2 launch roboone_motion_node motion.launch.py
    ros2 launch roboone_motion_node motion.launch.py dry_run:=true   # バスを開かない

機体一式は roboone_bringup/launch/roboone.launch.py から呼ぶ。こちらは
motion だけを個別に触りたいとき用。

★このノードはサーボにトルクを入れる（allow_torque が既定 true）。起動する前に
  機体を安全な高さで支えるか、脚が何にも当たらない姿勢にしておくこと。

  ただし起動しただけでは動かない。require_home_before_arm が既定 true なので、
  /cmd_motion を 1 回受けるまで脱力のまま待つ（teleop の Options 長押しが
  home → /estop false の順に送る）。動き出すときも、実測姿勢から torque_on_time 秒
  かけて保持姿勢へ補間するので、いきなり跳ねることはない。

  機体を動かさずに通し確認だけしたいときは allow_torque:=false
  （バスは開いて読むが、トルクも位置指令も送らない）。バスも開きたくないなら
  dry_run:=true。実装中の検証はこのどちらかで走らせること
  （CLAUDE.md「実装中の検証ではトルクを入れない」）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('roboone_motion_node')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=os.path.join(share, 'config', 'motion_node.yaml'),
            description='motion ノードのパラメータ YAML'),
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description='true でシリアルバスを開かない（config と IK の通し確認用）'),
        DeclareLaunchArgument(
            'allow_torque', default_value='true',
            description='★サーボにトルクを入れる。false で「読むだけ」の通し確認になる'),

        Node(
            package='roboone_motion_node', executable='motion_node', name='motion',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'dry_run': LaunchConfiguration('dry_run')},
                {'allow_torque': LaunchConfiguration('allow_torque')},
            ]),
    ])
