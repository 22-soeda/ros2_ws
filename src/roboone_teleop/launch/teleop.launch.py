# -*- coding: utf-8 -*-
"""PS5 (DualSense) 無線操縦の launch — joy バックエンド + teleop ノード。

    ros2 launch roboone_teleop teleop.launch.py

引数:

    joy_backend:=game_controller|joy   既定 game_controller
        game_controller … SDL の GameController API。SDL が DualSense の対応表を
                          内蔵しているので、ボタン/軸の並びが機種非依存になる。
                          config/ps5_dualsense.yaml の index はこちら前提。
        joy             … 生の SDL Joystick。並びはカーネルドライバ任せ。
                          SDL が知らないコントローラを繋ぐときの逃げ道。
    config:=<path>                     割り当て YAML の差し替え
    device_id:=0                       コントローラが複数刺さっているとき
    autorepeat_rate:=20.0              ★0 にしないこと
        0 にするとスティックを動かさない限り /joy が来なくなり、teleop の無通信
        ウォッチドッグが「静止」と「Bluetooth 断」を区別できなくなる。
    overrides:=<path>                  自分用の差分 YAML (省略可)
        config の**上に**重ねる (後のファイルが勝つ)。変えたいキーだけ書けばよく、
        パッケージの config を汚さずに手元の値を持てる。docs/teleop_tuning.md §3。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _teleop_node(context, *args, **kwargs):
    """本体の teleop ノード。config の上に overrides を重ねる (後のファイルが勝つ)。"""
    params = [LaunchConfiguration('config').perform(context)]
    overrides = LaunchConfiguration('overrides').perform(context).strip()
    if overrides:
        params.append(os.path.expanduser(overrides))
    return [Node(package='roboone_teleop', executable='teleop_node', name='teleop',
                 parameters=params, output='screen')]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('roboone_teleop'), 'config', 'ps5_dualsense.yaml')

    backend = LaunchConfiguration('joy_backend')
    device_id = LaunchConfiguration('device_id')
    autorepeat = LaunchConfiguration('autorepeat_rate')

    joy_params = [{
        'device_id': device_id,
        # 不感帯は teleop 側で掛ける。joy 側でも切ると二重に効いて効きが読めない。
        'deadzone': 0.0,
        'autorepeat_rate': autorepeat,
        'sticky_buttons': False,
        'coalesce_interval_ms': 1,
    }]

    is_gc = PythonExpression(["'", backend, "' == 'game_controller'"])
    is_joy = PythonExpression(["'", backend, "' != 'game_controller'"])

    return LaunchDescription([
        DeclareLaunchArgument('joy_backend', default_value='game_controller'),
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('autorepeat_rate', default_value='20.0'),
        DeclareLaunchArgument('overrides', default_value=''),

        # joy 側は respawn する。Bluetooth は切れるものなので、落ちたら上げ直す。
        # 切れている間は teleop の無通信ウォッチドッグが脱力を掛けるので、
        # 勝手に上げ直しても安全側は壊れない。
        Node(package='joy', executable='game_controller_node', name='joy_node',
             parameters=joy_params, output='screen', condition=IfCondition(is_gc),
             respawn=True, respawn_delay=2.0),
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=joy_params, output='screen', condition=IfCondition(is_joy),
             respawn=True, respawn_delay=2.0),

        # teleop 本体は引数の確定後に組む (overrides の有無で parameters が変わる)
        OpaqueFunction(function=_teleop_node),
    ])
