# -*- coding: utf-8 -*-
"""静歩行の設定ごとに、足先が実機の脚で届くかを走査する。

static_gait.yaml の swing_height の表はこれで作った。前後・左右・斜め・切り返しの
指令を static_walk に流し、各時刻の足先 (骨盤から見た位置) を leg_service の
IK + 機構層 (reach.py) に通して、届かない時刻を数える。

    python3 src/roboone_viz/roboone_viz/static_reach.py \
        --swing-height 0.035,0.03,0.025,0.02 --com-offset 0.01,0.005,0,-0.005,-0.01

先に colcon build --packages-select roboone_kinematics (leg_service を使う)。
★足裏は水平で見ている (reach.py の注記。home_pose の rpy・body_pitch が 0 の前提)。
"""

import argparse
import itertools
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'roboone_walk_ref'))

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from roboone_viz.reach import LegReach
else:
    from .reach import LegReach

from roboone_walk_ref.static_walk import StaticGaitParams, StaticWalkEngine  # noqa: E402

DT = 0.005


def _on(vx, vy, t1=9.5):
    return lambda t: (vx, vy) if 0.5 <= t < t1 else (0.0, 0.0)


def _two(a, b):
    return lambda t: a if 0.5 <= t < 8.0 else (b if 8.0 <= t < 16.0 else (0.0, 0.0))


def profiles(vx, vy):
    """前後・左右・斜め 4 方向と、前後・左右の切り返し、前進 -> 斜め の 11 通り。

    motion ノードの起動時の門 (motion_config.cpp の checkStaticWalkEnvelope) が
    同じ組を回す。変えたら両方を揃える。
    """
    dx, dy = 0.8 * vx, 0.625 * vy     # 斜めは楕円制限の内側
    return {
        'fwd': _on(vx, 0.0), 'back': _on(-vx, 0.0),
        'left': _on(0.0, vy), 'right': _on(0.0, -vy),
        'diag_fl': _on(dx, dy), 'diag_fr': _on(dx, -dy),
        'diag_bl': _on(-dx, dy), 'diag_f2': _on(0.7 * vx, 0.75 * vy),
        'rev_x': _two((vx, 0.0), (-vx, 0.0)),
        'rev_y': _two((0.0, vy), (0.0, -vy)),
        'fwd_diag': _two((vx, 0.0), (0.7 * vx, -0.75 * vy)),
    }


def scan(reach, sp, t_end=26.0, every=2):
    """{シナリオ名: (届かない時刻の数, 最初の 1 点)}。全部届けば空。"""
    cache = {}

    def ok(side, x, y, z):
        k = (side, round(x, 1), round(y, 1), round(z, 1))
        if k not in cache:
            cache[k] = reach.ok(*k)
        return cache[k]

    bad = {}
    for name, fn in profiles(*sp.v_max).items():
        e = StaticWalkEngine(sp)
        n, first = 0, None
        for i in range(int(t_end / DT)):
            o = e.update(*fn(i * DT), DT)
            if i % every:
                continue
            for side, f in (('L', o.left_foot), ('R', o.right_foot)):
                rel = [(f[k] - o.pelvis[k]) * 1000.0 for k in range(3)]
                if not ok(side, *rel):
                    n += 1
                    if first is None:
                        first = (round(o.t, 2), o.state, side,
                                 round(rel[0]), round(rel[1]), round(f[2] * 1000.0))
        if n:
            bad[name] = (n, first)
    return bad


def main(argv=None):
    ap = argparse.ArgumentParser(description='静歩行の設定ごとに足先が届くかを走査する')
    ap.add_argument('--static-gait', default=None, help='元にする static_gait.yaml')
    ap.add_argument('--swing-height', default=None, help='足上げ [m] (カンマ区切り)')
    ap.add_argument('--com-offset', default=None, help='重心の横ずらし [m] (+ で外側。カンマ区切り)')
    ap.add_argument('--vy-max', default=None, help='横の最高速 [m/s] (カンマ区切り)')
    ap.add_argument('-v', '--verbose', action='store_true', help='届かない最初の 1 点を出す')
    args = ap.parse_args(argv)

    base_path = args.static_gait or (
        Path(__file__).resolve().parents[2] / 'roboone_walk_ref' / 'config' / 'static_gait.yaml')
    base = StaticGaitParams.from_yaml(str(base_path))

    def axis(s, default):
        return [float(v) for v in s.split(',')] if s else [default]

    reach = LegReach.find()
    if reach is None:
        print('leg_service が見つからない (colcon build --packages-select roboone_kinematics)')
        return 1
    try:
        for h, off, vy in itertools.product(
                axis(args.swing_height, base.swing_height),
                axis(args.com_offset, base.com_offset_y),
                axis(args.vy_max, base.v_max[1])):
            sp = StaticGaitParams(**dict(base.to_dict(), swing_height=h, com_offset_y=off,
                                         v_max=(base.v_max[0], vy)))
            bad = scan(reach, sp)
            total = sum(v[0] for v in bad.values())
            head = f'h={h * 1000:4.0f}mm  offset={off * 1000:+4.0f}mm  vy_max={vy:.3f}'
            print(f'{head}  届かない時刻 {total:5d}  ' +
                  ('全時刻届く' if not bad else ' '.join(sorted(bad))), flush=True)
            if args.verbose:
                for name, (n, first) in sorted(bad.items()):
                    print(f'    {name:9s} {n:4d}  最初: t={first[0]} {first[1]} {first[2]}脚 '
                          f'骨盤から ({first[3]}, {first[4]}) mm 足上げ {first[5]} mm')
    finally:
        reach.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
