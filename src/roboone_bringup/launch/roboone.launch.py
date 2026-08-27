# -*- coding: utf-8 -*-
"""機体をひとまとめに立ち上げる launch。

    ros2 launch roboone_bringup roboone.launch.py

各パッケージの launch はそのまま残してある。このファイルが持つのは「どれを、
どの順で、どの引数で上げるか」だけ。個別に触りたいときは元の launch を直接叩く。

引数:

    ui:=true|false        ui ノード（OLED / RGB LED / ブザー）
    teleop:=true|false    joy + teleop ノード
    camera:=false|true    RealSense。既定 OFF（USB 帯域と CPU を食うので、要るときだけ）
    record:=false|true    ros2 bag 記録（config/record_topics.yaml のトピック）
    log_dir:=<path>       記録の置き場。既定 ~/roboone_logs

    joy_backend:=game_controller|joy    teleop へそのまま渡す
    teleop_config:=<path>               teleop の割り当て YAML

起動順と落ち方の設計:

  * **ui を先に上げる。** teleop の状態表示は latched なので順序に厳密な依存は
    ないが、ui が先にいれば起動直後から「まだ /joy が来ていない」が画面に出る。
    立ち上げ中に何も表示されない時間を作らない。
  * **どのノードが落ちても launch 全体は落とさない。** 特に teleop が死んだときに
    全部道連れにすると、記録が切れて原因が追えなくなり、OLED も消えて操作者が
    状況を読めなくなる。teleop は終了時にゼロ指令と /estop true を置いていくので、
    teleop だけが死んでも機体は脱力して止まる（motion 側の /cmd_walk タイムアウトも
    効く）。**止めるより、止まったことが分かる状態を残すほうを採る。**
  * teleop が死ぬと OLED は最後の RELAX 表示のまま固まり、LED は赤の点滅で残る。
    ターミナルには "process has died" が出る。この 3 つで気付ける前提。

motion ノードができたら、teleop の下に 1 ブロック足す（README「motion を足すとき」）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _record_topics():
    """記録するトピック一覧を config から読む。"""
    path = os.path.join(
        get_package_share_directory('roboone_bringup'), 'config', 'record_topics.yaml')
    with open(path) as f:
        return list(yaml.safe_load(f)['topics'])


def _recorder(context, *args, **kwargs):
    """記録プロセス。出力先に時刻を入れたいので引数の確定後に組み立てる。"""
    if LaunchConfiguration('record').perform(context).lower() not in ('true', '1'):
        return []
    log_dir = LaunchConfiguration('log_dir').perform(context)
    os.makedirs(log_dir, exist_ok=True)
    # 出力名は ros2 bag の既定 (rosbag2_<日時>) に任せ、cwd だけ指定する。
    # launch 側で時刻を作ると、同じ秒に 2 回起動したときに名前が衝突する。
    return [ExecuteProcess(
        cmd=['ros2', 'bag', 'record'] + _record_topics(),
        cwd=log_dir, output='screen')]


def generate_launch_description():
    teleop_share = get_package_share_directory('roboone_teleop')

    return LaunchDescription([
        DeclareLaunchArgument('ui', default_value='true',
                              description='ui ノード（OLED / RGB LED / ブザー）'),
        DeclareLaunchArgument('teleop', default_value='true',
                              description='joy + teleop ノード'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='RealSense。USB 帯域と CPU を食うので既定 OFF'),
        DeclareLaunchArgument('record', default_value='false',
                              description='ros2 bag 記録（config/record_topics.yaml）'),
        DeclareLaunchArgument('log_dir',
                              default_value=os.path.expanduser('~/roboone_logs'),
                              description='記録の置き場'),
        DeclareLaunchArgument('joy_backend', default_value='game_controller',
                              description='teleop へ渡す。game_controller | joy'),
        DeclareLaunchArgument(
            'teleop_config',
            default_value=os.path.join(teleop_share, 'config', 'ps5_dualsense.yaml'),
            description='teleop の割り当て YAML'),

        # --- 1) 表示を最初に上げる ------------------------------------------
        # ui ノードはデバイス単位で初期化失敗を握り潰すので、OLED が挿さって
        # いないだけで launch 全体が止まることはない。
        Node(package='roboone_ui', executable='ui_node', name='ui',
             output='screen', condition=IfCondition(LaunchConfiguration('ui'))),

        # --- 2) 操作系 ------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(teleop_share, 'launch', 'teleop.launch.py')),
            condition=IfCondition(LaunchConfiguration('teleop')),
            launch_arguments={
                'joy_backend': LaunchConfiguration('joy_backend'),
                'config': LaunchConfiguration('teleop_config'),
            }.items()),

        # --- 3) motion（未実装）---------------------------------------------
        # ここに roboone_motion のノードが入る。/cmd_walk・/cmd_motion・/estop の
        # 受け手なので、teleop より後で構わない（/estop は latched なので、後から
        # 上げても直前の脱力状態が届く）。

        # --- 4) カメラ（要るときだけ）----------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('realsense_bringup'),
                             'launch', 'realsense.launch.py')),
            condition=IfCondition(LaunchConfiguration('camera'))),

        # --- 5) 記録 ---------------------------------------------------------
        OpaqueFunction(function=_recorder),
    ])
