"""RealSense D435if を depth + IMU 構成で起動する。

    # 既定: depth 848x480x30 + IMU 200Hz + XYZ点群、RGB は OFF
    ros2 launch realsense_bringup realsense.launch.py

    # RGB も出す（点群に色が付く。USB帯域と CPU を食うので必要なときだけ）
    ros2 launch realsense_bringup realsense.launch.py enable_color:=true

    # 点群が要らない場合（depth 画像だけ使う / CPU を空けたい）
    ros2 launch realsense_bringup realsense.launch.py enable_pointcloud:=false

出るトピック（docs/ros-architecture.md のトピック表に合わせてある）:
    /camera/imu                        sensor_msgs/Imu          200Hz  → imu_filter
    /camera/depth/color/points         sensor_msgs/PointCloud2   30Hz  → opponent_detector
    /camera/depth/image_rect_raw       sensor_msgs/Image         30Hz
    /camera/color/image_raw            sensor_msgs/Image         30Hz  (enable_color:=true のとき)

パラメータの中身は config/realsense.yaml。この launch はストリームの ON/OFF と、
そこから決まる値（点群のテクスチャ元・フレーム同期）だけを面倒みる。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ROS2 の起動引数は文字列で来るので、bool として解釈しなおす
_TRUE = ("true", "1", "yes", "on")

# stream_filter: 点群にどのストリームをテクスチャとして貼るか
#   0 = RS2_STREAM_ANY  … テクスチャなし。XYZ だけの点群
#   2 = RS2_STREAM_COLOR … カラー画像を貼る（enable_color:=true が前提）
_PC_TEXTURE_NONE = 0
_PC_TEXTURE_COLOR = 2

# 点群フィルタのパラメータ接頭辞。
#
# realsense2_camera はフィルタのパラメータ名を librealsense のフィルタ名から
# 機械的に作る。ところが librealsense は点群フィルタに SIMD バックエンド名を
# 付けるので、この Pi (aarch64) では "Pointcloud (NEON)" → pointcloud__neon_
# になり、公式ドキュメントや rs_launch.py が使う "pointcloud" では効かない
# （起動ログに "Failed to get parameters: pointcloud.enable" だけ出て、
#  点群トピックが黙って作られない、という形で刺さる）。
#
# 実際の名前は起動中に `ros2 param list /camera | grep point` で確認できる。
# x86 のビルドでは "pointcloud"、GPU 加速(GLSL)版では別名になる。
_PC_PREFIX_DEFAULT = "pointcloud__neon_"


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return arg(name).lower() in _TRUE

    pc = arg("pointcloud_ns")
    enable_color = flag("enable_color")
    enable_imu = flag("enable_imu")
    enable_pointcloud = flag("enable_pointcloud")

    # 色付き点群を作るときだけ、depth と color を同じフレームセットに揃える必要がある。
    # RGB を使わないなら同期は不要（余計なバッファ遅延を作らない）。
    sync_frames = enable_color and enable_pointcloud

    params = {
        # frame_id の接頭辞。ノード名と揃えておかないと TF とトピックがずれる
        "camera_name": arg("camera_name"),
        "serial_no": arg("serial_no"),
        # 起動時にカメラをハードリセットする。既定 true。
        #
        # RSUSB バックエンドでは、ノードを Ctrl-C 以外で落とすと IMU (HID) の
        # エンドポイントが握られたままになり、次の起動で「Motion Module: Starting /
        # Open profile Accel,Gyro」までログに出るのに /camera/imu が1つも流れない、
        # という状態に落ちることがある（エラーは何も出ないので気づきにくい）。
        # リセットすると確実に直るので、起動が数秒延びる代わりに既定で入れてある。
        # 起動時間を詰めたい場面では initial_reset:=false にできる。
        "initial_reset": flag("initial_reset"),

        "enable_depth": True,
        "enable_color": enable_color,
        # 赤外の生画像は使わない（帯域と CPU の無駄）
        "enable_infra": False,
        "enable_infra1": False,
        "enable_infra2": False,

        # IMU。gyro と accel の両方が要る（片方だけだと /camera/imu が作られない）
        "enable_gyro": enable_imu,
        "enable_accel": enable_imu,

        # 点群。接頭辞が環境依存なので、yaml ではなくここでまとめて組み立てる
        pc + ".enable": enable_pointcloud,
        pc + ".stream_filter": (
            _PC_TEXTURE_COLOR if (enable_pointcloud and enable_color) else _PC_TEXTURE_NONE
        ),
        # ordered_pc=false: 無効画素を落とした 1行 N点の非順序点群。
        # opponent_detector は画素の並びを使わないので、点数が減る分こちらが軽い。
        pc + ".ordered_pc": False,
        # texture が取れない画素も点として残すか。RGB OFF 時は texture 自体を
        # 使わないので影響しない。
        pc + ".allow_no_texture_points": False,
        "enable_sync": sync_frames,
        # depth を color に射影しなおした画像は使わないので OFF（CPU を食う）
        "align_depth.enable": False,
        "enable_rgbd": False,
    }

    config = os.path.join(
        get_package_share_directory("realsense_bringup"), "config", "realsense.yaml")

    return [
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace=arg("camera_namespace"),
            name=arg("camera_name"),
            # 後勝ち: yaml を土台にして、この launch が決めた ON/OFF で上書きする
            parameters=[config, params],
            output="screen",
            emulate_tty=True,
            arguments=["--ros-args", "--log-level", arg("log_level")],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_color", default_value="false",
            description="RGB ストリームを出すか。既定 false（使う予定がなく、USB帯域を空けるため）"),
        DeclareLaunchArgument(
            "enable_imu", default_value="true",
            description="gyro+accel を出し、統合した /camera/imu を publish するか"),
        DeclareLaunchArgument(
            "enable_pointcloud", default_value="true",
            description="/camera/depth/color/points を出すか。RGB OFF なら XYZ のみの点群になる"),
        DeclareLaunchArgument(
            "pointcloud_ns", default_value=_PC_PREFIX_DEFAULT,
            description="点群フィルタのパラメータ接頭辞。この Pi では pointcloud__neon_ "
                        "（`ros2 param list /camera | grep point` で確認できる）"),
        DeclareLaunchArgument(
            "camera_name", default_value="camera",
            description="ノード名 兼 frame_id の接頭辞。トピックは /<camera_namespace>/<camera_name>/... になる"),
        DeclareLaunchArgument(
            "camera_namespace", default_value="",
            description="名前空間。既定は空で、トピックが /camera/imu 等になる"),
        DeclareLaunchArgument(
            "serial_no", default_value="",
            description="カメラが複数あるときシリアル番号で選ぶ（例: '327122072324'）"),
        DeclareLaunchArgument(
            "initial_reset", default_value="true",
            description="起動時にカメラをハードリセットする。既定 true"),
        DeclareLaunchArgument(
            "log_level", default_value="info",
            description="[DEBUG|INFO|WARN|ERROR|FATAL]"),
        OpaqueFunction(function=launch_setup),
    ])
