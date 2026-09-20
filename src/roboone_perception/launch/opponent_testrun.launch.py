# -*- coding: utf-8 -*-
r"""相手機の認識のテストラン。検出器 + リアルタイムのビューアを立ち上げる。

    # カメラごと (ロボット無しで確認できる形。サーボには触れない)
    ros2 launch roboone_perception opponent_testrun.launch.py camera:=true

    # カメラは別で上げてある
    ros2 launch roboone_perception opponent_testrun.launch.py

    # 1 フレームごとの判定を CSV に残す
    ros2 launch roboone_perception opponent_testrun.launch.py camera:=true log:=/tmp/opp.csv

    # ★機体をホーム姿勢でゆっくり立たせてから見る (トルクが入る。機体を支えておくこと)
    ros2 launch roboone_perception opponent_testrun.launch.py camera:=true home:=true
    ros2 launch roboone_perception opponent_testrun.launch.py camera:=true home:=true \
        home_time:=10.0

起動時に表示される URL (既定 8105 番) を手元の PC のブラウザで開く。
確かめる順は docs/相手機の認識.md §9。

opponent_viewer は opponent_detector ノードそのものに HTTP の口を付けたもので、
/opponent と /ring_edge も publish する。**opponent_detector.launch.py と同時には
上げないこと** (/opponent が二重になる)。

home:=true について
-------------------
カメラは胴体に付いているので、本番と同じ高さ・向きで見るには機体を立たせる必要がある。
**既定は false で、motion ノードを上げず、サーボへ何も書かない。** true のときだけ

  * motion ノードをここから上げる。武装の補間時間 torque_on_time (既定 2 s) と
    home_move_time (既定 1.5 s) を home_time 秒 (既定 6 s) に延ばすので、脱力して
    いる今の姿勢から**ゆっくり**ホーム姿勢へ移る
  * ビューアが motion の起動を /motion/state で確かめ、home_delay 秒 (既定 3 s) 後に
    /cmd_motion "home" → /estop false を送る (teleop の Options 長押しと同じ順)
  * 画面に「脱力」「ホームへ」のボタンが出る。Ctrl-C でも脱力する (ビューアが
    /estop true を置き、motion も終了時に脱力する)。ビューアが落ちたら launch ごと
    畳むので、立たせたまま操作の口が無くなることはない

**roboone.launch.py と同時に上げないこと。** motion ノードが 2 つになりサーボのバスを
取り合う。teleop も上がらないので、コントローラの脱力ボタン (L1) は効かない。
トルクを入れずに流れだけ確かめるなら allow_torque:=false を足す。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('roboone_perception')
    default_config = os.path.join(share, 'config', 'opponent_detector.yaml')
    motion_share = get_package_share_directory('roboone_motion')
    home_time = ParameterValue(LaunchConfiguration('home_time'), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera', default_value='false',
            description='RealSense も一緒に上げるか。既に上がっているなら false'),
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='opponent_detector のパラメータ YAML'),
        DeclareLaunchArgument('port', default_value='8105',
                              description='ビューアの HTTP ポート'),
        DeclareLaunchArgument('log', default_value='',
                              description='1 フレームごとの判定を残す CSV (空なら残さない)'),
        DeclareLaunchArgument(
            'home', default_value='false', choices=['true', 'false'],
            description='★true でトルクを入れ、ホーム姿勢へゆっくり立たせる。'
                        '既定 false (motion を上げず、サーボへ何も書かない)'),
        DeclareLaunchArgument(
            'home_time', default_value='6.0',
            description='home:=true のとき、今の姿勢からホーム姿勢へ移るのにかける時間 [s]'),
        DeclareLaunchArgument(
            'home_delay', default_value='3.0',
            description='home:=true のとき、motion の起動を確かめてからトルクを入れるまでの待ち [s]'),
        DeclareLaunchArgument(
            'allow_torque', default_value='true',
            description='home:=true のときだけ効く。false でトルクも位置指令も送らず、'
                        '流れだけ確かめる'),
        DeclareLaunchArgument(
            'motion_config',
            default_value=os.path.join(motion_share, 'config', 'motion_node.yaml'),
            description='home:=true のときの motion ノードのパラメータ YAML'),

        LogInfo(condition=IfCondition(LaunchConfiguration('home')),
                msg='★★ home:=true — トルクが入る。機体を支えておくこと ★★'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('realsense_bringup'),
                             'launch', 'realsense.launch.py')),
            condition=IfCondition(LaunchConfiguration('camera')),
            launch_arguments={
                'enable_imu': 'true',
                'enable_pointcloud': 'false',
            }.items()),

        # motion ノード (home:=true のときだけ)。motion.launch.py を include せず
        # 直接書いているのは、武装の補間時間を上書きしたいから。上書きは yaml の後ろに
        # 置いた辞書が勝つ。
        Node(package='roboone_motion', executable='motion_node', name='motion',
             output='screen',
             condition=IfCondition(LaunchConfiguration('home')),
             parameters=[
                 LaunchConfiguration('motion_config'),
                 {'allow_torque': LaunchConfiguration('allow_torque')},
                 {'torque_on_time': home_time},
                 {'home_move_time': home_time},
             ]),

        # ビューアが落ちたら launch ごと畳む。home:=true で立たせたまま、脱力の口
        # (画面のボタン) だけが無くなる状態を作らない。motion は終了時に脱力する。
        Node(package='roboone_perception', executable='opponent_viewer',
             name='opponent_detector', output='screen', emulate_tty=True,
             parameters=[LaunchConfiguration('config')],
             arguments=['--port', LaunchConfiguration('port'),
                        '--log', LaunchConfiguration('log'),
                        '--home-delay', LaunchConfiguration('home_delay'),
                        PythonExpression([
                            "'--home' if '", LaunchConfiguration('home'),
                            "' == 'true' else '--no-home'"])],
             on_exit=Shutdown()),
    ])
