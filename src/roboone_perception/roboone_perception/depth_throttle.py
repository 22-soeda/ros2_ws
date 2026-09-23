# -*- coding: utf-8 -*-
r"""depth_throttle — depth を間引いて別のトピックへ流す。bag に録るためだけの道具。

**なぜ要るか。** 848x480 の 16UC1 は 1 枚 814 KB で、30 Hz なら 24.4 MB/s になる。
この Pi の SD カードは実測 22.5 MB/s なので、生のまま `ros2 bag record` に渡すと
書き込みが追いつかずに取りこぼす。

カメラ側を 15 Hz に落とせば収まるが、**それでは診断したい対象そのものが変わる**
(検出器が受け取るフレーム数も姿勢の引き戻しの回数も変わる)。そこで、カメラは
30 Hz のまま回しておいて、**bag に入れる分だけ**ここで間引く。

    /camera/depth/image_rect_raw  --(every N)-->  /rec/depth/image_rect_raw
    /camera/depth/camera_info     --(同じ回)-->   /rec/depth/camera_info

IMU (/camera/imu) は 1 サンプルが小さいので間引かずにそのまま録る。

読むだけで、サーボにもパラメータにも触れない。

    ros2 run roboone_perception depth_throttle              # 3 枚に 1 枚 (10 Hz)
    ros2 run roboone_perception depth_throttle --every 6    # 5 Hz
    ros2 run roboone_perception depth_throttle --hz 5       # 枚数ではなく Hz で指定

再生するときは名前を戻す:

    ros2 bag play <bag> --remap /rec/depth/image_rect_raw:=/camera/depth/image_rect_raw \\
                                /rec/depth/camera_info:=/camera/depth/camera_info

あるいは読む側に直接渡す (height_probe / detector_bench はどちらも受ける):

    ros2 run roboone_perception height_probe \\
        --depth-topic /rec/depth/image_rect_raw --info-topic /rec/depth/camera_info
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image


class Throttle(Node):
    """N 枚に 1 枚だけ流す。camera_info は流した回だけ付ける。"""

    def __init__(self, args):
        super().__init__('depth_throttle')
        self.every = max(1, int(args.every))
        self.n_in = 0
        self.n_out = 0
        self.info = None
        self.t0 = time.monotonic()
        qos = QoSProfile(depth=2, reliability=(
            ReliabilityPolicy.BEST_EFFORT if args.best_effort
            else ReliabilityPolicy.RELIABLE))
        self.pub_depth = self.create_publisher(Image, args.out_depth, qos)
        self.pub_info = self.create_publisher(CameraInfo, args.out_info, qos)
        self.create_subscription(Image, args.depth_topic, self._depth, qos)
        self.create_subscription(CameraInfo, args.info_topic, self._info, qos)
        self.create_timer(5.0, self._tick)
        self.get_logger().info(
            'depth を %d 枚に 1 枚だけ %s へ流す (camera_info も同じ回)'
            % (self.every, args.out_depth))

    def _info(self, msg):
        self.info = msg

    def _depth(self, msg):
        self.n_in += 1
        if (self.n_in - 1) % self.every:
            return
        self.pub_depth.publish(msg)
        if self.info is not None:
            self.pub_info.publish(self.info)
        self.n_out += 1

    def _tick(self):
        dt = time.monotonic() - self.t0
        if dt <= 0 or self.n_in == 0:
            self.get_logger().warn('depth が来ていない')
            return
        self.get_logger().info(
            '受け %d 枚 (%.1f Hz) / 出し %d 枚 (%.1f Hz)'
            % (self.n_in, self.n_in / dt, self.n_out, self.n_out / dt))


def main(argv=None):
    ap = argparse.ArgumentParser(description='depth を間引いて bag 用に流す')
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--every', type=int, default=3, help='N 枚に 1 枚 (既定 3 = 10 Hz)')
    g.add_argument('--hz', type=float, default=None,
                   help='欲しい周期 [Hz]。--src-hz から --every を決める')
    ap.add_argument('--src-hz', type=float, default=30.0, help='元の depth の周期 [Hz]')
    ap.add_argument('--depth-topic', default='/camera/depth/image_rect_raw')
    ap.add_argument('--info-topic', default='/camera/depth/camera_info')
    ap.add_argument('--out-depth', default='/rec/depth/image_rect_raw')
    ap.add_argument('--out-info', default='/rec/depth/camera_info')
    ap.add_argument('--best-effort', action='store_true')
    # launch から起動されると --ros-args が付く。argparse に食わせると
    # 終了コード 2 で落ちるので、先に ROS の引数を外す (2026-09-23)
    raw = list(sys.argv if argv is None else [sys.argv[0]] + list(argv))
    args = ap.parse_args(remove_ros_args(raw)[1:])
    if args.hz is not None and args.hz > 0:
        args.every = max(1, int(round(args.src_hz / args.hz)))

    rclpy.init(args=raw)
    node = Throttle(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
