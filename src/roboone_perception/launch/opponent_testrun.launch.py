# -*- coding: utf-8 -*-
r"""相手機の認識のテストラン。検出器 + リアルタイムのビューアを立ち上げる。

    # カメラごと (ロボット無しで確認できる形。サーボには触れない)
    ros2 launch roboone_perception opponent_testrun.launch.py camera:=true

    # カメラは別で上げてある
    ros2 launch roboone_perception opponent_testrun.launch.py

    # 1 フレームごとの判定を CSV に残す
    ros2 launch roboone_perception opponent_testrun.launch.py camera:=true log:=/tmp/opp.csv

    # depth ごと bag に録る (後から実機なしで同じ入力を流し直せる)
    ros2 launch roboone_perception opponent_testrun.launch.py camera:=true bag:=true

    # ★機体をホーム姿勢でゆっくり立たせてから見る (トルクが入る。機体を支えておくこと)
    ros2 launch roboone_perception opponent_testrun.launch.py camera:=true home:=true
    ros2 launch roboone_perception opponent_testrun.launch.py camera:=true home:=true \
        home_time:=10.0
    ros2 launch roboone_perception opponent_testrun.launch.py camera:=true home:=true \
        walk_mode:=static

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
  * **足首による姿勢の安定化 (stab) もそのまま効く。** motion_config (既定
    roboone_motion/config/motion_node.yaml) の stab.* をそのまま使い、HOLD で
    立っている間も補正が入る (motion_node.cpp の layer_active は HOLD と WALK)。
    入力は /camera/imu なので、camera:=true か、別で上げた RealSense が IMU を
    出していること。見るのは ros2 topic echo /motion/stab
  * 立位は **walk_mode ごとに違う** (home_pose.yaml の walk_mode:)。motion.launch.py
    と違ってここは既定 dynamic を渡すので、普段 static で動かしているなら
    walk_mode:=static を足す

**roboone.launch.py と同時に上げないこと。** motion ノードが 2 つになりサーボのバスを
取り合う。teleop も上がらないので、コントローラの脱力ボタン (L1) は効かない。
トルクを入れずに流れだけ確かめるなら allow_torque:=false を足す。

bag:=true について
------------------
その場で見ているものを**そのまま録って、後から実機なしで流し直せる**ようにする。
画面で症状が出たときに既に録れている、という形にしたいので、テストランに混ぜてある。

**depth はそのままでは録れない。** 848x480 の 16UC1 は 1 枚 814 KB で 30 Hz なら
24.4 MB/s、この Pi の SD カードは実測 22.5 MB/s で追いつかない。カメラ側を 15 Hz に
落とすと診断したい対象そのものが変わるので、**カメラも検出器も 30 Hz のまま回して、
bag に入れるぶんだけ** depth_throttle で間引く (既定 10 Hz ≒ 8 MB/s)。録れるのは

  /rec/depth/image_rect_raw  /rec/depth/camera_info   (間引いた depth)
  /camera/imu                                          (そのまま)
  /opponent  /ring_edge                                (そのときの判断)
  /motion/state  /joint_states  /autonomy  /estop      (機体の状態)

出力先は既定で ~/roboone_logs/opp_<日時>。Ctrl-C で閉じる。再生は

    ros2 bag play <bag> --loop
    ros2 run roboone_perception height_probe \
        --depth-topic /rec/depth/image_rect_raw --info-topic /rec/depth/camera_info
"""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, Shutdown)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


#: bag に録るトピック。depth だけ depth_throttle が間引いたものを使う
#: (生の 30 Hz は 24.4 MB/s で、この Pi の SD カード 22.5 MB/s に入らない)。
#: 検出器の出力も一緒に録る。後で「そのとき何と判断したか」と突き合わせるため
_BAG_TOPICS = ['/rec/depth/image_rect_raw', '/rec/depth/camera_info',
               '/camera/imu', '/opponent', '/ring_edge',
               '/motion/state', '/joint_states', '/autonomy', '/estop']


def generate_launch_description():
    share = get_package_share_directory('roboone_perception')
    default_config = os.path.join(share, 'config', 'opponent_detector.yaml')
    motion_share = get_package_share_directory('roboone_motion')
    home_time = ParameterValue(LaunchConfiguration('home_time'), value_type=float)
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    default_bag = os.path.join(os.path.expanduser('~'), 'roboone_logs',
                               'opp_' + stamp)

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
        DeclareLaunchArgument(
            'depth_profile', default_value='',
            description='深度のプロファイル (例 848x480x15)。**空 = yaml のまま '
                        '(848x480x30)**。検出が 1 フレームの予算を超えるときの逃げ道で、'
                        '既定では使わない。先に tune.cell を粗くすること'),
        DeclareLaunchArgument(
            'bag', default_value='true', choices=['true', 'false'],
            description='depth を間引いて bag に録る。実機なしで後から同じ入力を'
                        '流し直せるようになる。**既定 true** (2026-09-24)。'
                        '10 Hz で約 8 MB/s = 毎分 470 MB 書くので、容量が'
                        '気になるときは bag:=false か bag_hz:=5.0'),
        DeclareLaunchArgument(
            'bag_hz', default_value='10.0',
            description='bag に入れる depth の周期 [Hz]。10 Hz で約 8 MB/s。'
                        'この Pi の SD カードは実測 22.5 MB/s なので 20 Hz が上限の目安'),
        DeclareLaunchArgument(
            'bag_dir', default_value=default_bag,
            description='bag の出力先。既定は ~/roboone_logs/opp_<日時>'),
        DeclareLaunchArgument(
            'walk_mode', default_value='dynamic', choices=['dynamic', 'static'],
            description='home:=true のときの歩行の計画器。**立位がモードごとに違う**'
                        '（home_pose.yaml の walk_mode:）ので、普段の launch と'
                        '揃えること。motion.launch.py の既定と同じ dynamic'),

        LogInfo(condition=IfCondition(LaunchConfiguration('home')),
                msg='★★ home:=true — トルクが入る。機体を支えておくこと ★★'),
        LogInfo(condition=UnlessCondition(LaunchConfiguration('home')),
                msg='home:=false — motion ノードを上げないので**サーボは動かない**。'
                    '立たせて見るなら home:=true を足す（トルクが入る）'),
        LogInfo(condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration('home'), "' == 'true' and '",
                    LaunchConfiguration('camera'), "' != 'true'"])),
                msg='安定化 (stab) は /camera/imu を読む。camera:=false なので、'
                    '別で上げている RealSense が IMU を出していないと効かない'),
        LogInfo(condition=IfCondition(LaunchConfiguration('bag')),
                msg=['bag を録る -> ', LaunchConfiguration('bag_dir'),
                     ' (depth は ', LaunchConfiguration('bag_hz'),
                     ' Hz に間引く。Ctrl-C で閉じる)']),

        # bag 用に depth を間引く。**カメラも検出器も 30 Hz のまま**で、
        # 録るぶんだけ減らす (診断したい挙動そのものを変えないため)
        Node(package='roboone_perception', executable='depth_throttle',
             name='depth_throttle', output='screen',
             condition=IfCondition(LaunchConfiguration('bag')),
             arguments=['--hz', LaunchConfiguration('bag_hz')]),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('bag')),
            cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_dir'),
                 '--max-cache-size', '134217728'] + _BAG_TOPICS,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('realsense_bringup'),
                             'launch', 'realsense.launch.py')),
            condition=IfCondition(LaunchConfiguration('camera')),
            launch_arguments={
                'enable_imu': 'true',
                'enable_pointcloud': 'false',
                'depth_profile': LaunchConfiguration('depth_profile'),
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
                 {'walk_mode': LaunchConfiguration('walk_mode')},
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
