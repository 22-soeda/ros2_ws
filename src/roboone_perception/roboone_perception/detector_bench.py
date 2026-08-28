# -*- coding: utf-8 -*-
"""detector_bench — 実機の depth で検出器を回して、周期・段ごとの時間・検出率を測る。

docs/opponent_detection.pdf §10 と §12 の「Pi 5 での実測をしていない」を埋めるための道具。
文書の時間はすべて AMD Ryzen 7 8845HS 上の値で、Pi 5 の Cortex-A76 は単スレッドで
これより遅い。30 Hz の予算 33 ms に収まるかは、ここで測ってから決める。

    ros2 run roboone_perception detector_bench                  # 10 秒
    ros2 run roboone_perception detector_bench --duration 30
    ros2 run roboone_perception detector_bench --stride 3       # §10 の 1 番目の手

realsense_bringup の test ツールと同じ方針で、**判定が通れば終了コード 0、
外れれば 1** にしてある（起動確認のゲートに使える）。

判定:
  * depth が来ていること
  * 1 フレームの処理時間の p95 が予算 (既定 33 ms) に収まっていること
  * リング面が取れたフレームの割合が下限 (既定 80%) を超えていること

最後の 2 つは「カメラがリングを向いていること」が前提なので、机に伏せた状態で
走らせれば当然落ちる。0.5〜3 m 先にリングに見立てた床が見える向きで使う。
"""

import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu

from .detect import DetectorParams, Intrinsics, OK, RING_LOST, RingDetector

_STAGES = ('deproject', 'attitude', 'plane', 'grid', 'edge', 'cluster', 'track',
           'total')


class Bench(Node):

    def __init__(self, args):
        super().__init__('detector_bench')
        self.args = args
        flat = {'tune.stride': args.stride, 'body.cam_height': args.cam_height,
                'body.cam_pitch_deg': args.cam_pitch}
        self.det = RingDetector(DetectorParams.from_flat(flat))
        self.intr = None
        self.prev = None
        self.imu = []
        self.accel = None
        self.rows = []
        self.n_depth = 0
        self.gaps = []

        qos = QoSProfile(depth=2, reliability=(
            ReliabilityPolicy.BEST_EFFORT if args.best_effort
            else ReliabilityPolicy.RELIABLE))
        self.create_subscription(CameraInfo, args.info_topic, self._info, qos)
        self.create_subscription(Image, args.depth_topic, self._depth, qos)
        self.create_subscription(
            Imu, args.imu_topic, self._imu,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _info(self, msg):
        self.intr = Intrinsics.from_camera_info(msg)

    def _imu(self, msg):
        w = msg.angular_velocity
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imu.append((t, (w.x, w.y, w.z)))
        a = msg.linear_acceleration
        self.accel = (a.x, a.y, a.z)

    def _depth(self, msg):
        self.n_depth += 1
        if self.intr is None:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev is not None:
            self.gaps.append(stamp - self.prev)
        gyro = []
        last = self.prev if self.prev else stamp
        for t, w in self.imu:
            if self.prev is None or not (self.prev < t <= stamp):
                continue
            dt = t - last
            if 0.0 < dt < 0.1:
                gyro.append((w, dt))
            last = t
        self.imu = [s for s in self.imu if s[0] > stamp]
        dt = (stamp - self.prev) if self.prev else 1.0 / 30.0
        self.prev = stamp

        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height,
                                                                 msg.width)
        res = self.det.step(depth, self.intr, dt, gyro=gyro, accel=self.accel)
        self.rows.append((res.status, res.n_points, res.ring_area,
                          res.plane_resid, res.plane_corrected,
                          dict(res.timings)))


def _pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float('nan')


def report(bench, args):
    rows = bench.rows
    print()
    print('=== detector_bench (%.1f s / stride=%d) ===' % (args.duration, args.stride))
    if not rows:
        print('  NG  depth を 1 枚も処理できなかった (受信 %d 枚, intrinsics %s)'
              % (bench.n_depth, 'あり' if bench.intr else 'なし'))
        print('      ! QoS 不一致なら --best-effort を試す。トピック名は --depth-topic')
        return False

    ok = True
    gaps = np.array(bench.gaps) if bench.gaps else np.array([0.0])
    hz = 1.0 / gaps.mean() if gaps.mean() > 0 else 0.0
    print('depth        %d 枚 / %.2f Hz / 最大間隔 %.1f ms'
          % (len(rows), hz, gaps.max() * 1e3))
    print('点数         平均 %d 点' % np.mean([r[1] for r in rows]))

    print()
    print('  段            mean [ms]   p95 [ms]    max [ms]')
    for st in _STAGES:
        v = np.array([r[5].get(st, 0.0) for r in rows]) * 1e3
        mark = ''
        if st == 'total':
            mark = '  <= 予算 %.0f ms' % args.budget
            if _pct(v, 95) > args.budget:
                mark = '  ! 予算 %.0f ms 超過' % args.budget
                ok = False
        print('  %-12s %8.2f %10.2f %11.2f%s'
              % (st, v.mean(), _pct(v, 95), v.max(), mark))

    n = len(rows)
    got_ring = sum(1 for r in rows if r[0] != RING_LOST)
    got_opp = sum(1 for r in rows if r[0] == OK)
    corrected = sum(1 for r in rows if r[4])
    area = np.array([r[2] for r in rows])
    resid = np.array([r[3] for r in rows if np.isfinite(r[3])])
    print()
    print('リング面取得 %5.1f %%  (面積 mean %.2f m^2 / p10 %.2f)'
          % (100 * got_ring / n, area.mean(), _pct(area, 10)))
    print('姿勢の引き戻し %3.1f %%  (あてはめ残差 mean %.1f mm / p95 %.1f mm)'
          % (100 * corrected / n, resid.mean() * 1e3 if resid.size else float('nan'),
             _pct(resid, 95) * 1e3 if resid.size else float('nan')))
    print('相手あり     %5.1f %%  (status=OK のフレーム)' % (100 * got_opp / n))
    if 100 * got_ring / n < args.min_ring:
        print('  ! リング面が取れたフレームが %.0f %% を下回った'
              ' (カメラが床を向いているか確認する)' % args.min_ring)
        ok = False
    print()
    print('判定: %s' % ('OK' if ok else 'NG'))
    return ok


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--duration', type=float, default=10.0, help='測定時間 [s]')
    p.add_argument('--stride', type=int, default=2, help='間引き幅 d (§4 表 2)')
    p.add_argument('--budget', type=float, default=33.0,
                   help='1 フレームの処理時間の予算 [ms]。30Hz なら 33')
    p.add_argument('--min-ring', type=float, default=80.0,
                   help='リング面が取れたフレームの下限 [%%]')
    p.add_argument('--cam-height', type=float, default=0.35, help='h_cam [m]')
    p.add_argument('--cam-pitch', type=float, default=30.0, help='俯角 [deg]')
    p.add_argument('--depth-topic', default='/camera/depth/image_rect_raw')
    p.add_argument('--info-topic', default='/camera/depth/camera_info')
    p.add_argument('--imu-topic', default='/camera/imu')
    p.add_argument('--best-effort', action='store_true',
                   help='depth を BEST_EFFORT で購読する')
    args = p.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = Bench(args)
    print('%.0f 秒測ります… (0.5〜3 m 先に床が見える向きで)' % args.duration)
    end = time.time() + args.duration
    try:
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    ok = report(node, args)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
