#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ros2 bag の /motion/stab から、歩行区間ごとのロールの揺れをまとめる。

    python3 scripts/bag_walk_roll.py ~/roboone_logs/rosbag2_2026_09_17-01_41_58
    python3 scripts/bag_walk_roll.py <bag> --series 19 27        # その時刻の時系列を出す
    python3 scripts/bag_walk_roll.py <bag> --series 19 27 --dt 0.02

前提: `source /opt/ros/jazzy/setup.bash`（rosbag2_py と型定義が要る）。
時刻は bag の先頭からの秒。

===========================================================================
何を見るためのものか
===========================================================================
単脚支持の間に、胴体が**どちらへ**どれだけ傾いたか。歩ごとに

    遊脚側への傾き = support × roll     （support: +1 左足支持 / -1 右足支持、
                                          roll: + で右へ倒れている）

の最大を出す。正なら遊脚側（持ち上げた足の側）へ、負なら支持足の外側へ倒れている。

    正で歩ごとに育つ -> 骨盤の横振りが足りない（gait.yaml の foot_spacing と
                        motion_node.yaml の stance_y_offset。2026-09-17）
    負             -> 振りすぎ

`/motion/stab` の並びは layout のラベルから引くので、列を足しても壊れない。
"""

from __future__ import annotations

import argparse
import math
import sys

R2D = 180.0 / math.pi
WALKING = (1, 2, 3)     # walk_state: START / STEP / STOP


def read_stab(path):
    """(bag 先頭からの秒, {ラベル: 値}) のリストと /cmd_walk を返す。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float64MultiArray

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/motion/stab', '/cmd_walk']))
    t0 = None
    rows, cmds = [], []
    labels = None
    while r.has_next():
        topic, data, t = r.read_next()
        t *= 1e-9
        if t0 is None:
            t0 = t
        if topic == '/cmd_walk':
            m = deserialize_message(data, Twist)
            cmds.append((t - t0, m.linear.x, m.linear.y))
            continue
        m = deserialize_message(data, Float64MultiArray)
        if labels is None:
            labels = m.layout.dim[0].label.split(',')
        rows.append((t - t0, dict(zip(labels, m.data))))
    return rows, cmds


def segments(rows):
    """walk_state が歩行中の連続区間 [(開始 index, 終了 index)]。"""
    out, a = [], None
    for i, (_, s) in enumerate(rows):
        w = int(s['walk_state']) in WALKING
        if w and a is None:
            a = i
        if not w and a is not None:
            out.append((a, i - 1))
            a = None
    if a is not None:
        out.append((a, len(rows) - 1))
    return out


def per_step_tilt(rows, a, b):
    """単脚支持の 1 区間ごとの「遊脚側への傾き」の最大 [deg]。"""
    out, cur, sup = [], None, 0
    for _, s in rows[a:b + 1]:
        sp = int(s['support'])
        if sp != sup:
            if cur is not None:
                out.append(cur)
            cur, sup = None, sp
        if sp != 0:
            v = sp * s['roll'] * R2D
            cur = v if cur is None else max(cur, v)
    if cur is not None:
        out.append(cur)
    return out


def summary(rows, cmds):
    for a, b in segments(rows):
        ta, tb = rows[a][0], rows[b][0]
        if tb - ta < 0.5:
            continue
        steps = per_step_tilt(rows, a, b)
        roll = [abs(s['roll']) * R2D for _, s in rows[a:b + 1]]
        after = [abs(s['roll']) * R2D for t, s in rows if tb < t < tb + 2.0]
        ur = max(abs(s['u_roll']) * R2D for _, s in rows[a:b + 1])
        c = [x for x in cmds if ta <= x[0] <= tb]
        vx = max((abs(x[1]) for x in c), default=0.0)
        vy = max((abs(x[2]) for x in c), default=0.0)
        print(f'歩行 {ta:8.2f}-{tb:8.2f}s ({tb - ta:5.1f}s) {len(steps):2d} 歩  '
              f'|vx|≤{vx:.2f} |vy|≤{vy:.2f}  |roll| 最大 {max(roll):5.1f}° '
              f'停止後 2s {max(after, default=0.0):5.1f}°  |u_roll| 最大 {ur:4.1f}°')
        if steps:
            print('    遊脚側への傾き [deg] 歩ごと: ' + ' '.join(f'{v:+.0f}' for v in steps))


def series(rows, t0, t1, dt):
    print('     t   walk sup   φ     roll°  pitch°   gx°/s  u_roll°  足首θ6 R/L°')
    last = -1e9
    for t, s in rows:
        if t < t0 or t > t1 or t - last < dt - 1e-9:
            continue
        last = t
        print(f'{t:8.2f}  {s["walk_state"]:3.0f} {s["support"]:+3.0f} {s["phase"]:5.2f} '
              f'{s["roll"] * R2D:+7.1f} {s["pitch"] * R2D:+7.1f} {s["gyro_x"] * R2D:+7.1f} '
              f'{s["u_roll"] * R2D:+7.1f}  {s["ank_R_th6"] * R2D:+5.1f}/{s["ank_L_th6"] * R2D:+5.1f}')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--series', nargs=2, type=float, metavar=('T0', 'T1'),
                    help='この区間の時系列を出す [s]')
    ap.add_argument('--dt', type=float, default=0.05, help='--series の間引き [s]')
    args = ap.parse_args()
    rows, cmds = read_stab(args.bag)
    if not rows:
        print('/motion/stab が入っていない', file=sys.stderr)
        return 1
    if args.series:
        series(rows, args.series[0], args.series[1], args.dt)
    else:
        summary(rows, cmds)
    return 0


if __name__ == '__main__':
    sys.exit(main())
