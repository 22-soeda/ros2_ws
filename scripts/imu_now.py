#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""いまの胴体のロール・ピッチを /motion/stab から平均して出す。

    python3 scripts/imu_now.py              # 5 秒ぶん平均して 1 回出す
    python3 scripts/imu_now.py --sec 20     # 20 秒ぶん
    python3 scripts/imu_now.py --watch      # 1 秒ごとに出し続ける（Ctrl-C で止める）

前提: `source /opt/ros/jazzy/setup.bash` と、motion ノードが動いていること
（/motion/stab は motion ノードが出す。トルクは入っていなくてよい）。

===========================================================================
何のためのものか
===========================================================================
「立たせると左へ 1.8 度傾く」のような**一定の偏り**を、数字で持ち帰るためのもの。
床・機体・IMU の取り付けのどれが効いているかを切り分けるのに、同じ姿勢で
向きを変えながら何度も測る（docs/commands.md「立位の左右差を切り分ける」）。

  roll  + で右へ倒れている
  pitch + で前へ倒れている

平均と一緒に標準偏差も出す。σ が大きいときは機体が揺れているので、
静止してから測り直すこと（静止していれば σ は 0.05 deg 以下になる）。
"""

from __future__ import annotations

import argparse
import math
import statistics as st
import sys

R2D = 180.0 / math.pi
I_ROLL, I_PITCH, I_AT_REST = 4, 5, 10


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sec', type=float, default=5.0, help='平均する秒数 (既定 5)')
    ap.add_argument('--watch', action='store_true', help='1 秒ごとに出し続ける')
    ap.add_argument('--label', default='', help='行頭に付ける覚え書き (向きなど)')
    ap.add_argument('--load', action='store_true',
                    help='脚の荷重の左右差も出す（IMU に依らない裏取り）')
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float64MultiArray
    from sensor_msgs.msg import JointState

    buf: list[tuple[float, float, float]] = []
    leg: dict[str, list[tuple[float, float]]] = {}   # 軸 -> [(偏差deg, 負荷%)]
    LEG_IDS = ('ID1', 'ID2', 'ID3', 'ID4', 'ID5', 'ID6')
    CNT2DEG = 360.0 / 4096.0

    class Sub(Node):
        def __init__(self):
            super().__init__('imu_now')
            # ★/motion/stab は SensorDataQoS (BEST_EFFORT) で出ている。既定の
            #   RELIABLE で購読すると「incompatible QoS」で 1 通も来ない
            #   (motion_node.cpp の pub_stab_)
            self.create_subscription(
                Float64MultiArray, '/motion/stab', self.cb, qos_profile_sensor_data)
            if a.load:
                # /motion/servo_states は既定の QoS（RELIABLE, depth 10）
                self.create_subscription(JointState, '/motion/servo_states', self.cb_srv, 10)

        def cb(self, m):
            if len(m.data) > I_AT_REST:
                buf.append((m.data[I_ROLL], m.data[I_PITCH], m.data[I_AT_REST]))

        def cb_srv(self, m):
            for n, pos, vel, eff in zip(m.name, m.position, m.velocity, m.effort):
                if n[2:] in LEG_IDS and n[:2] in ('L_', 'R_'):
                    leg.setdefault(n, []).append(((pos - vel) * CNT2DEG, abs(eff) * 100.0))

    def report(win: float) -> None:
        if len(buf) < 2:
            print('/motion/stab が来ていない（motion ノードは動いていますか）')
            return
        roll = [x[0] * R2D for x in buf]
        pitch = [x[1] * R2D for x in buf]
        rest = sum(1 for x in buf if x[2] > 0.5) / len(buf)
        head = (a.label + '  ') if a.label else ''
        print('%sroll %+6.2f (σ %.2f)  pitch %+6.2f (σ %.2f)  静止 %3.0f%%  %d サンプル / %.0fs'
              % (head, st.mean(roll), st.pstdev(roll), st.mean(pitch), st.pstdev(pitch),
                 100 * rest, len(buf), win))
        if not a.load:
            return
        if not leg:
            print('  /motion/servo_states が来ていない')
            return
        tot = {}
        for s in ('L', 'R'):
            v = [x for k, r in leg.items() if k[0] == s for x in r]
            tot[s] = st.mean(x[1] for x in v) if v else float('nan')
        print('  脚の平均負荷  左 %.1f%%  右 %.1f%%   （差 %+.1f ポイント。'
              '偏りが大きい側に体重が乗っている）' % (tot['L'], tot['R'], tot['L'] - tot['R']))
        print('  %-8s %9s %7s | %-8s %9s %7s' % ('軸', '偏差°', '負荷%', '軸', '偏差°', '負荷%'))
        for idn in LEG_IDS:
            cols = []
            for s in ('L', 'R'):
                v = leg.get(s + '_' + idn)
                cols.append((s + '_' + idn,
                             st.median([x[0] for x in v]) if v else float('nan'),
                             st.median([x[1] for x in v]) if v else float('nan')))
            print('  %-8s %9.2f %7.1f | %-8s %9.2f %7.1f'
                  % (cols[0][0], cols[0][1], cols[0][2],
                     cols[1][0], cols[1][1], cols[1][2]))

    rclpy.init()
    n = Sub()
    try:
        if a.watch:
            while True:
                buf.clear(); leg.clear()
                rclpy.spin_once(n, timeout_sec=0.0)
                t_end = n.get_clock().now().nanoseconds + 1_000_000_000
                while n.get_clock().now().nanoseconds < t_end:
                    rclpy.spin_once(n, timeout_sec=0.05)
                report(1.0)
        else:
            t_end = n.get_clock().now().nanoseconds + int(a.sec * 1e9)
            while n.get_clock().now().nanoseconds < t_end:
                rclpy.spin_once(n, timeout_sec=0.05)
            report(a.sec)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
