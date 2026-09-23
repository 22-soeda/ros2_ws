# -*- coding: utf-8 -*-
"""height_probe — depth と IMU から「床は実際どこにあるか」を逆算する。

検出器が `RING_LOST` (リング面が取れない) に落ちると、**欲しい数字だけが出なくなる。**
テストランの画面の「カメラ高さ 実測」は `-h_r` なので、`h_r` が決まらないと空欄になる。
ところが原因の切り分けに要るのはまさにその値、という噛み合わせの悪さがある
(docs/相手機の認識_実機テストランの課題.md の観測 2)。

そこでこの道具は**検出器の状態を一切使わない**:

  * 鉛直は **IMU の加速度**から取る (既定)。検出器がジャイロで運んでいる漂った u は
    使わない。`--vertical mount` にすると取り付け (body.cam_pitch_deg) から取る
  * 高さの窓 (`tune.hist_window`) で絞らずに、**広い範囲のヒストグラムをそのまま出す**。
    床がどこにあるかを人が見て決められる

読むだけで、サーボにもパラメータにも触れない。実機を立たせたまま並行して走らせてよい
(depth と IMU の購読が 1 つ増えるだけ)。

    ros2 run roboone_perception height_probe                   # 3 秒
    ros2 run roboone_perception height_probe --duration 10
    ros2 run roboone_perception height_probe --vertical mount  # 取り付けから鉛直を取る
    ros2 run roboone_perception height_probe --cam-height 0.42 # 別の設定値と比べる

判定 (終了コード): 床の山が見つかって、検出器の窓にも入っていれば 0、外れていれば 1。
"""

import argparse
import math
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image, Imu

import yaml

from .detect import (Deprojector, DetectorParams, forward_ref, Intrinsics,
                     ring_basis, to_plane)
from .detect import ring as rg


def default_config():
    """入っている opponent_detector.yaml の場所。"""
    try:
        return os.path.join(get_package_share_directory('roboone_perception'),
                            'config', 'opponent_detector.yaml')
    except Exception:                   # pragma: no cover
        return ''


def flat_from_yaml(path):
    """YAML の body.* / match.* / tune.* を平らな辞書にする。

    **検出器が実際に使っている値と比べるため**にここを読む。params.py の
    dataclass の既定値と比べても、この機体での設定とは限らない。
    """
    try:
        with open(path, encoding='utf-8') as f:
            doc = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return {}
    params = None
    for node in doc.values():
        if isinstance(node, dict) and isinstance(node.get('ros__parameters'), dict):
            params = node['ros__parameters']
            break
    if params is None:
        return {}
    out = {}
    for grp in ('body', 'match', 'tune'):
        for k, v in (params.get(grp) or {}).items():
            out['%s.%s' % (grp, k)] = v
    return out


class Probe(Node):
    """depth・camera_info・IMU を集めるだけのノード。計算は最後にまとめてやる。"""

    def __init__(self, args):
        super().__init__('height_probe')
        self.args = args
        self.intr = None
        self.frames = []
        self.accel = []
        self.gyro = []
        self.n_depth = 0
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
        a, w = msg.linear_acceleration, msg.angular_velocity
        self.accel.append((a.x, a.y, a.z))
        self.gyro.append(math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z))

    def _depth(self, msg):
        self.n_depth += 1
        if self.intr is None or len(self.frames) >= self.args.max_frames:
            return
        self.frames.append(np.frombuffer(msg.data, dtype=np.uint16)
                           .reshape(msg.height, msg.width).copy())


def _histogram(h, lo, hi, bin_w):
    nbins = max(1, int(round((hi - lo) / bin_w)))
    counts, edges = np.histogram(h, bins=nbins, range=(lo, hi))
    return counts, edges


def _draw(counts, edges, width=44, keep=0.02):
    """テキストのヒストグラム。山の周りだけ出す (全部出すと読めない)。"""
    top = int(counts.max()) if counts.size else 0
    if top == 0:
        return ['  (範囲の中に点が無い)']
    peak = int(np.argmax(counts))
    shown = np.flatnonzero(counts >= max(1, keep * top))
    lo_i, hi_i = int(shown.min()), int(shown.max())
    out = []
    for i in range(lo_i, hi_i + 1):
        c = 0.5 * (edges[i] + edges[i + 1])
        bar = '#' * int(round(width * counts[i] / top))
        out.append('  %+.3f |%-*s %6d%s'
                   % (c, width, bar, counts[i], '  <- 最頻' if i == peak else ''))
    if len(out) > 40:                       # 長すぎるときは山の周りだけ
        k = out[max(0, peak - lo_i - 20):peak - lo_i + 20]
        out = ['  ... (上略)'] + k + ['  ... (下略)']
    return out


def build_params(args):
    """YAML を土台にして、明示された引数だけ上書きする。"""
    flat = flat_from_yaml(args.config) if args.config else {}
    if args.cam_height is not None:
        flat['body.cam_height'] = args.cam_height
    if args.cam_pitch is not None:
        flat['body.cam_pitch_deg'] = args.cam_pitch
    if args.stride is not None:
        flat['tune.stride'] = args.stride
    return DetectorParams.from_flat(flat), flat


def report(probe, args):
    p, flat = build_params(args)
    t, b = p.tune, p.body
    print()
    print('=== height_probe (%.1f s) ===' % args.duration)
    print('設定           %s'
          % (args.config if flat else '(yaml を読めず、params.py の既定値を使う)'))
    if probe.intr is None or not probe.frames:
        print('  NG  depth を 1 枚も受け取れなかった (受信 %d 枚, intrinsics %s)'
              % (probe.n_depth, 'あり' if probe.intr else 'なし'))
        print('      ! QoS が違うなら --best-effort。トピック名は --depth-topic')
        return False

    # --- 鉛直 u ---------------------------------------------------------
    if args.vertical == 'accel':
        if not probe.accel:
            print('  NG  IMU が来ていない。--vertical mount なら取り付けから取れる')
            return False
        acc = np.mean(np.asarray(probe.accel, dtype=np.float64), axis=0)
        norm = float(np.linalg.norm(acc))
        if norm < 1e-3:
            print('  NG  加速度がほぼ 0。IMU を確かめること')
            return False
        u = acc / norm
        gmax = max(probe.gyro) if probe.gyro else 0.0
        still = gmax < 0.15 and abs(norm - 9.81) < 0.6
        src = ('加速度から (|a| = %.2f m/s^2, 角速度の最大 %.2f rad/s, %s)'
               % (norm, gmax, '静止' if still else '**動いている**'))
    else:
        u = np.asarray(b.up_from_mount, dtype=np.float64)
        still = True
        src = '取り付けから (cam_pitch_deg = %.1f)' % b.cam_pitch_deg
    mount = np.asarray(b.up_from_mount, dtype=np.float64)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(u, mount))))))

    # --- 逆投影して高さを出す -------------------------------------------
    depth = probe.frames[len(probe.frames) // 2]        # 真ん中の 1 枚
    deproj = Deprojector(probe.intr, t.stride, t.border_px)
    pts, _ = deproj(depth, args.depth_scale, t.depth_min, t.depth_max)
    if pts.shape[0] == 0:
        print('  NG  有効な深度が 1 点も無い (%.2f〜%.2f m の外)'
              % (t.depth_min, t.depth_max))
        return False
    e1, e2 = ring_basis(u, forward_ref(b.cam_pitch_deg))
    h, fwd, left = to_plane(pts, u, e1, e2)

    print('鉛直 u        %s' % src)
    print('              取り付けからのずれ %.1f deg' % tilt)
    print('depth         %d 枚 (使ったのは 1 枚) / %d 点 (stride=%d, %.2f〜%.2f m)'
          % (len(probe.frames), pts.shape[0], t.stride, t.depth_min, t.depth_max))
    print()
    print('高さヒストグラム (u に沿った、カメラ原点からの高さ [m]。%.0f cm 刻み。'
          '負 = 下)' % (args.bin * 100))
    counts, edges = _histogram(h, args.lo, args.hi, args.bin)
    for line in _draw(counts, edges):
        print(line)

    if counts.max() == 0:
        print()
        print('  NG  %.2f〜%.2f m に点が無い。--lo / --hi を広げて見ること'
              % (args.lo, args.hi))
        return False

    # --- 山の高さと、そこへの平面あてはめ --------------------------------
    peak = 0.5 * (edges[np.argmax(counts)] + edges[np.argmax(counts) + 1])
    near = h[np.abs(h - peak) < t.hist_refine]
    h_peak = float(near.mean()) if near.size else float(peak)
    fit = rg.fit_plane(pts, h, fwd, left, h_peak, t.fit_band, t.fit_radius,
                       t.fit_min_points, t.fit_max_points)
    print()
    print('最頻の山      %+.3f m  →  **カメラ高さ 実測 %.3f m**' % (h_peak, -h_peak))
    if fit.ok:
        ang = math.degrees(math.acos(
            max(-1.0, min(1.0, abs(float(np.dot(fit.normal, u)))))))
        print('平面あてはめ  残差 %.1f mm / 法線と鉛直の食い違い %.1f deg'
              % (fit.resid * 1e3, ang))
        print('              (門は 残差 %.0f mm / %.0f deg。どちらか外れると姿勢が劣化する)'
              % (t.fit_resid_max * 1e3, t.fit_angle_max_deg))
    else:
        ang = float('nan')
        print('平面あてはめ  **点が足りない** (水平 %.1f m 以内・帯 ±%.0f mm に %d 点未満)'
              % (t.fit_radius, t.fit_band * 1e3, t.fit_min_points))

    # --- 検出器の窓に入るか ---------------------------------------------
    lo, hi = -b.cam_height - t.hist_window, -b.cam_height + t.hist_window
    n_win = int(np.count_nonzero((h > lo) & (h < hi)))
    inside = n_win >= t.hist_min_points
    print()
    print('設定値        body.cam_height = %.3f m   (実測との差 %+.3f m)'
          % (b.cam_height, -h_peak - b.cam_height))
    print('検出器の窓    %+.3f 〜 %+.3f m に %d 点 (下限 %d) → %s'
          % (lo, hi, n_win, t.hist_min_points, '通る' if inside else '**通らない**'))
    print('床が写り始める距離  %.2f m (高さ %.3f / tan %.0f deg)'
          % (b.cam_height / max(1e-6, math.tan(math.radians(29.0))),
             b.cam_height, 29.0))

    # --- 読み方 ----------------------------------------------------------
    print()
    ok = inside and fit.ok
    if not still:
        print('  ! 動いている間の加速度は鉛直として当てにならない。静止させて測り直す')
    if not inside:
        print('  NG  床の山が検出器の窓の外にある。これが「リング面が取れない」の中身。')
        print('      * 実測 %.3f m が設定 %.3f m から離れているなら body.cam_height を直す'
              % (-h_peak, b.cam_height))
        print('      * 離れていないのに窓を外すなら、検出器の鉛直が漂っている。')
        print('        ホーム姿勢で静止して /detector/reset_attitude を送る')
    elif fit.ok and ang > t.fit_angle_max_deg:
        print('  NG  床の法線が鉛直と %.1f deg ずれている (門は %.0f deg)。'
              % (ang, t.fit_angle_max_deg))
        print('      機体が傾いているか、見ているのが床ではない')
    else:
        print('  OK  床は見えていて、検出器の窓にも入っている。')
        if abs(-h_peak - b.cam_height) > 0.02:
            print('      ただし body.cam_height を %.3f にすると窓の中心が合う'
                  % round(-h_peak, 3))
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(description='床の高さ (= カメラ高さ) を逆算する')
    ap.add_argument('--duration', type=float, default=3.0, help='集める秒数')
    ap.add_argument('--max-frames', type=int, default=30, help='貯める depth の枚数')
    ap.add_argument('--vertical', choices=['accel', 'mount'], default='accel',
                    help='鉛直の取り方。既定は IMU の加速度 (検出器の u は使わない)')
    ap.add_argument('--config', default=default_config(),
                    help='比べる相手の yaml。既定は入っている opponent_detector.yaml')
    ap.add_argument('--cam-height', type=float, default=None,
                    help='比較する設定値 [m]。既定は yaml の body.cam_height')
    ap.add_argument('--cam-pitch', type=float, default=None,
                    help='取り付けの俯角 [deg]。既定は yaml の body.cam_pitch_deg')
    ap.add_argument('--stride', type=int, default=None,
                    help='深度の間引き。既定は yaml の tune.stride')
    ap.add_argument('--lo', type=float, default=-1.5, help='ヒストグラムの下端 [m]')
    ap.add_argument('--hi', type=float, default=0.5, help='ヒストグラムの上端 [m]')
    ap.add_argument('--bin', type=float, default=0.02, help='ヒストグラムの刻み [m]')
    ap.add_argument('--depth-scale', type=float, default=0.001)
    ap.add_argument('--depth-topic', default='/camera/depth/image_rect_raw')
    ap.add_argument('--info-topic', default='/camera/depth/camera_info')
    ap.add_argument('--imu-topic', default='/camera/imu')
    ap.add_argument('--best-effort', action='store_true',
                    help='depth を BEST_EFFORT で購読する')
    # launch から起動されると --ros-args が付く。argparse に食わせると
    # 終了コード 2 で落ちるので、先に ROS の引数を外す (2026-09-23)
    raw = list(sys.argv if argv is None else [sys.argv[0]] + list(argv))
    args = ap.parse_args(remove_ros_args(raw)[1:])

    rclpy.init(args=raw)
    node = Probe(args)
    t0 = time.monotonic()
    try:
        while (time.monotonic() - t0 < args.duration and rclpy.ok()):
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    ok = False
    try:
        ok = report(node, args)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
