#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ros2 bag から「指令と実測の差（沈み込み）」を取り出して表にする。

    python3 scripts/bag_droop.py ~/roboone_logs/rosbag2_2026_08_28-19_30_00
    python3 scripts/bag_droop.py <bag> --state HOLD      # その状態の区間だけ見る
    python3 scripts/bag_droop.py <bag> --csv out.csv     # 時系列を CSV で出す

前提: `source /opt/ros/jazzy/setup.bash`（rosbag2_py と型定義が要る）。

===========================================================================
何を見るためのものか
===========================================================================
「浮かせたときと接地したときで姿勢が違う」＝ 荷重に応じた定常偏差（droop）。
これは指令と実測を突き合わせないと見えない。bag には

    /joint_states          実測の関節角 [rad]
    /motion/joint_commands 指令の関節角 [rad]
    /motion/servo_states   サーボ空間（position=実測カウント / velocity=目標カウント
                           / effort=負荷）
    /motion/diagnostics    電圧・温度・応答軸数
    /motion/state          そのときの状態（RELAX / HOLD / WALK / MOTION:...）

が入っているので、ここから軸ごとの偏差を出す。

**関節空間とサーボ空間の両方を出す理由**: 間に膝 4 節リンクの変換が挟まっている。

    サーボ空間に差がある -> サーボが指令位置を保持できていない（P/I ゲインの話）
    サーボ空間は一致、関節空間だけ差 -> リンク・フレームのたわみ（ゲインでは直らない）

負荷（effort）も併記する。負荷が上限に張り付いていなければ、トルク上限ではなく
ゲインの問題（サーボは「偏差 x P」ぶんしかトルクを出そうとしない）。
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import math
import sys

R2D = 180.0 / math.pi
CNT2DEG = 360.0 / 4096.0


def read_bag(path):
    """bag を舐めて (topic, 受信時刻[s], メッセージ) を順に返す。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
        rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in types:
            continue
        yield topic, stamp * 1e-9, deserialize_message(data, get_message(types[topic]))


def nearest(series, t):
    """時刻 t 以前で最も近いサンプルを返す（無ければ None）。"""
    lo, hi, best = 0, len(series) - 1, None
    while lo <= hi:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            best = series[mid]
            lo = mid + 1
        else:
            hi = mid - 1
    return best


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--state', help='この /motion/state の区間だけ見る（前方一致）')
    ap.add_argument('--csv', help='時系列をこのファイルへ CSV で出す')
    args = ap.parse_args()

    meas, cmd, servo, diag, states = [], [], [], [], []
    try:
        for topic, t, m in read_bag(args.bag):
            if topic == '/joint_states':
                meas.append((t, dict(zip(m.name, m.position))))
            elif topic == '/motion/joint_commands':
                cmd.append((t, dict(zip(m.name, m.position))))
            elif topic == '/motion/servo_states':
                servo.append((t, m))
            elif topic == '/motion/diagnostics':
                diag.append((t, m))
            elif topic == '/motion/state':
                states.append((t, m.data))
    except Exception as exc:                                     # noqa: BLE001
        print(f'bag を読めない: {exc}', file=sys.stderr)
        return 2

    if not meas or not cmd:
        print('/joint_states と /motion/joint_commands が両方要る。'
              'record_topics.yaml を確認すること。', file=sys.stderr)
        return 1

    def state_at(t):
        s = nearest(states, t)
        return s[1] if s else '?'

    def wanted(t):
        return args.state is None or state_at(t).startswith(args.state)

    # --- 関節空間の偏差 ---------------------------------------------------
    acc = defaultdict(list)
    rows = []
    for t, mm in meas:
        c = nearest(cmd, t)
        if c is None or not wanted(t):
            continue
        for name, mv in mm.items():
            if name not in c[1]:
                continue
            d = (mv - c[1][name]) * R2D
            acc[name].append(d)
            rows.append((t, name, c[1][name] * R2D, mv * R2D, d, state_at(t)))

    if not acc:
        print('条件に合うサンプルが無い（--state を外すか、記録を確認）', file=sys.stderr)
        return 1

    span = f'（{args.state} の区間のみ）' if args.state else ''
    print(f'=== 関節空間の偏差 実測 - 指令 [deg] {span} ===')
    print(f'{"関節":<16}{"平均":>9}{"最大":>9}{"最小":>9}{"標本":>7}')
    for name in sorted(acc, key=lambda k: -abs(sum(acc[k]) / len(acc[k]))):
        v = acc[name]
        print(f'{name:<16}{sum(v)/len(v):>9.2f}{max(v):>9.2f}{min(v):>9.2f}{len(v):>7}')

    # --- サーボ空間の偏差と負荷 -------------------------------------------
    if servo:
        sacc, lacc = defaultdict(list), defaultdict(list)
        for t, m in servo:
            if not wanted(t):
                continue
            for i, name in enumerate(m.name):
                pos, goal = m.position[i], m.velocity[i]
                if pos != pos:            # NaN = 応答が無かった軸
                    continue
                sacc[name].append((pos - goal) * CNT2DEG)
                if i < len(m.effort):
                    lacc[name].append(abs(m.effort[i]) * 100.0)
        if sacc:
            print(f'\n=== サーボ空間の偏差 実測 - 目標 [deg] と負荷 [%] {span} ===')
            print(f'{"軸":<10}{"平均偏差":>10}{"最大偏差":>10}{"平均負荷":>10}{"最大負荷":>10}')
            for name in sorted(sacc, key=lambda k: -abs(sum(sacc[k]) / len(sacc[k]))):
                v, ld = sacc[name], lacc.get(name, [0.0])
                print(f'{name:<10}{sum(v)/len(v):>10.2f}{max(v, key=abs):>10.2f}'
                      f'{sum(ld)/len(ld):>10.1f}{max(ld):>10.1f}')
            print('\n負荷が上限(98%)に張り付いていなければ、トルク上限ではなくゲインの問題。')

    # --- 電圧 -------------------------------------------------------------
    if diag:
        volts = defaultdict(list)
        for t, m in diag:
            for st in m.status:
                for kv in st.values:
                    if kv.key.startswith('電圧'):
                        try:
                            volts[st.name].append(float(kv.value))
                        except ValueError:
                            pass
        print('\n=== 電圧 [V] ===')
        for k, v in volts.items():
            v = [x for x in v if x > 1.0]
            if v:
                print(f'  {k}: 平均 {sum(v)/len(v):.2f} / 最低 {min(v):.2f}')

    # --- 状態の遷移 -------------------------------------------------------
    if states:
        print('\n=== /motion/state の遷移 ===')
        t0 = states[0][0]
        prev = None
        for t, s in states:
            if s != prev:
                print(f'  {t - t0:8.2f}s  {s}')
                prev = s

    if args.csv:
        with open(args.csv, 'w') as f:
            f.write('t,joint,cmd_deg,meas_deg,droop_deg,state\n')
            for r in rows:
                f.write(f'{r[0]:.4f},{r[1]},{r[2]:.4f},{r[3]:.4f},{r[4]:.4f},{r[5]}\n')
        print(f'\n時系列を {args.csv} に書いた（{len(rows)} 行）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
