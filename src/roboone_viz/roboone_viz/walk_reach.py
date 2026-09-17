# -*- coding: utf-8 -*-
"""動歩行 (walk_core) の設定ごとに、横振り・発散・足先が届くかを走査する。

gait.yaml の ds_time (両足支持) の表はこれで作った。足踏み・前後・左右・斜め・
切り返しの指令を walk_core に流し、各時刻の足先を motion ノードと同じ変換
(計画の立位をホーム姿勢の足へ平行移動) で骨盤から見た位置に直して、leg_service の
IK + 機構層 (reach.py) に通す。

    python3 src/roboone_viz/roboone_viz/walk_reach.py \
        --ds-time 0,0.2,0.3,0.4 --foot-spacing 0.14,0.15 --swing-height 0.05,0.04,0.03

先に colcon build --packages-select roboone_kinematics (leg_service を使う)。
★足裏は水平で見ている (reach.py の注記。home_pose の rpy・body_pitch が 0 の前提)。
★足踏みは walk_core に無いので、前後の歩幅を 0 にした包み (March) で回す。
"""

import argparse
from dataclasses import replace
import itertools
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'roboone_walk_ref'))

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from roboone_viz.reach import LegReach
else:
    from .reach import LegReach

from roboone_walk_ref.walk_core import GaitParams, WalkEngine  # noqa: E402

DT = 0.005
CONFIG = Path(__file__).resolve().parents[2] / 'roboone_walk_ref' / 'config'


class March(WalkEngine):
    """足踏み: 前後の歩幅だけ 0 にする (前後の指令は歩き出し・歩き続けのスイッチになる)。"""

    def _step_params(self):
        return self._no_x(super()._step_params)

    def _step_params_ds(self):
        return self._no_x(super()._step_params_ds)

    def _no_x(self, fn):
        v0 = self.v[0]
        self.v[0] = 0.0
        try:
            return fn()
        finally:
            self.v[0] = v0


def _on(vx, vy):
    return lambda t: (vx, vy) if 0.5 <= t < 8.5 else (0.0, 0.0)


def _two(a, b):
    return lambda t: (0.0, 0.0) if t < 0.5 else (a if t < 5.5 else (b if t < 10.5 else (0.0, 0.0)))


def profiles(vx, vy):
    """(名前, 計画器, 指令, 長さ [s])。"""
    return [
        ('march', March, _on(vx, 0.0), 16.0),
        ('fwd', WalkEngine, _on(vx, 0.0), 16.0),
        ('back', WalkEngine, _on(-vx, 0.0), 16.0),
        ('left', WalkEngine, _on(0.0, vy), 16.0),
        ('right', WalkEngine, _on(0.0, -vy), 16.0),
        ('diag', WalkEngine, _on(0.8 * vx, 0.625 * vy), 16.0),
        ('rev_y', WalkEngine, _two((0.0, vy), (0.0, -vy)), 18.0),
        ('f_bdiag', WalkEngine, _two((vx, 0.0), (-vx, -vy)), 18.0),
        ('f_diag', WalkEngine, _two((vx, 0.0), (0.7 * vx, -0.75 * vy)), 18.0),
    ]


def home_half(path):
    """home_pose.yaml の foot.y [mm] (実機の足の半間隔)。"""
    import yaml
    with open(path, encoding='utf-8') as f:
        return float((yaml.safe_load(f) or {}).get('foot', {}).get('y', 89.3))


def scan(reach, p, real_half):
    """1 つの設定を走査する。

    戻り値 dict: inward (単脚支持の中央で骨盤が実機の支持足の中心から離れている量 [mm]。
    足踏みの定常)、vlat (同じく骨盤の横の最高速 [m/s])、diverged / clamped / bad (名前 -> 数)。
    """
    off = real_half - p.foot_spacing * 500.0
    cache = {}

    def ok(side, x, y, z):
        k = (side, round(x, 1), round(y, 1), round(z, 1))
        if k not in cache:
            cache[k] = reach.ok(*k)
        return cache[k]

    r = {'inward': float('nan'), 'vlat': float('nan'), 'diverged': [], 'clamped': [], 'bad': {}}
    for name, cls, fn, t_end in profiles(*p.v_max):
        e = cls(p)
        n = 0
        prev = None
        mids, speeds = [], []
        blown = False
        for i in range(int(t_end / DT)):
            t = i * DT
            o = e.update(*fn(t), DT)
            if abs(o.xi[0] - o.com[0]) > 0.3 or abs(o.xi[1] - o.com[1]) > 0.3:
                blown = True
                break
            if name == 'march' and 4.0 < t < 8.0:
                if prev is not None:
                    speeds.append(abs(o.com[1] - prev) / DT)
                if o.support != 0 and abs(o.phase - 0.5) < 1e-9:
                    f = o.left_foot if o.support > 0 else o.right_foot
                    real_y = f[1] + o.support * off / 1000.0
                    mids.append(abs(real_y - o.com[1]) * 1000.0)
            prev = o.com[1]
            if i % 2:
                continue
            fp = o.foot_targets_pelvis()
            for side, key, lat in (('R', 'right', -1), ('L', 'left', +1)):
                x, y, z = fp[key]
                if not ok(side, x * 1000.0, y * 1000.0 + lat * off, z * 1000.0):
                    n += 1
        if blown:
            r['diverged'].append(name)
            continue
        if name == 'march':
            r['inward'] = sum(mids) / len(mids) if mids else float('nan')
            r['vlat'] = max(speeds) if speeds else float('nan')
        if any(s.clamped for s in e.steps):
            r['clamped'].append(name)
        if n:
            r['bad'][name] = n
    return r


def main(argv=None):
    ap = argparse.ArgumentParser(description='動歩行の設定ごとに横振り・発散・足先の到達を走査する')
    ap.add_argument('--gait', default=str(CONFIG / 'gait.yaml'), help='元にする gait.yaml')
    ap.add_argument('--home-pose', default=str(CONFIG / 'home_pose.yaml'),
                    help='実機の足の位置 (foot.y) を読む home_pose.yaml')
    ap.add_argument('--ds-time', default=None, help='両足支持 [s] (カンマ区切り)')
    ap.add_argument('--foot-spacing', default=None, help='計画上の足間隔 [m] (カンマ区切り)')
    ap.add_argument('--swing-height', default=None, help='足上げ [m] (カンマ区切り)')
    args = ap.parse_args(argv)

    base = GaitParams.from_yaml(args.gait)
    real_half = home_half(args.home_pose)

    def axis(s, default):
        return [float(v) for v in s.split(',')] if s else [default]

    reach = LegReach.find()
    if reach is None:
        print('leg_service が見つからない (colcon build --packages-select roboone_kinematics)')
        return 1
    print(f'実機の足 ±{real_half:.1f}mm (home_pose.yaml)。届かない時間は 10ms 刻みで数えた合計')
    try:
        for td, w, h in itertools.product(
                axis(args.ds_time, base.ds_time),
                axis(args.foot_spacing, base.foot_spacing),
                axis(args.swing_height, base.swing_height)):
            p = replace(base, ds_time=td, foot_spacing=w, swing_height=h)
            r = scan(reach, p, real_half)
            bad = r['bad']
            print(f'ds={td:.2f}s W={w * 1000:5.1f}mm h={h * 1000:3.0f}mm | '
                  f'骨盤→支持足 {r["inward"]:5.1f}mm 横速 {r["vlat"]:.2f}m/s | '
                  f'発散 {",".join(r["diverged"]) or "-"} | '
                  f'クランプ {",".join(r["clamped"]) or "-"} | '
                  f'届かない {sum(bad.values()) * 10}ms '
                  + ' '.join(f'{k}={v * 10}ms' for k, v in bad.items()), flush=True)
    finally:
        reach.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
