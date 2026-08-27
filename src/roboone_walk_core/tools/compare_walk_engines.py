#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""walk_core の Python / C++ / JS 3 実装の数値照合。

Python 版 (roboone_motion) が仕様の原本。C++ 版 (walk_dump) と JS 版
(roboone_motion/viz/walkcore.js を node で実行) が同じ指令プロファイルで
同じ軌道を出すことを、tick ごとの最大絶対誤差で確認する。

使い方 (ws ルートから):
    python3 src/roboone_walk_core/tools/compare_walk_engines.py

libm と V8 の exp() の最終ビット差が e^{ωT} で増幅される分を見込み、
許容誤差は 1e-6 m (0.001 mm) とする。
"""

import math
from pathlib import Path
import subprocess
import sys

WS = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WS / 'src' / 'roboone_motion'))

from roboone_motion.walk_core import WalkEngine  # noqa: E402

DT = 0.005
TOL = 1e-6
CASES = [(0.10, 0.0), (-0.10, 0.0), (0.0, 0.06), (0.0, -0.06),
         (0.08, 0.05), (0.15, 0.08)]
COLS = ['t', 'st', 'ph', 'sup', 'vx', 'vy', 'xix', 'xiy', 'comx', 'comy',
        'zx', 'zy', 'lfx', 'lfy', 'lfz', 'rfx', 'rfy', 'rfz']
STATE_CODE = {'IDLE': 0, 'START': 1, 'STEP': 2, 'STOP': 3, 'ESTOP': 4}


def run_python(vx, vy, t_walk=4.5, t_end=8.0):
    e = WalkEngine()
    rows = []
    for i in range(int(t_end / DT + 0.5)):
        t = i * DT
        on = 0.5 <= t < t_walk
        o = e.update(vx if on else 0.0, vy if on else 0.0, DT)
        rows.append([o.t, STATE_CODE[o.state], o.phase, o.support,
                     o.v[0], o.v[1], o.xi[0], o.xi[1], o.com[0], o.com[1],
                     o.zmp[0], o.zmp[1], *o.left_foot, *o.right_foot])
    return rows


def parse_csv(text):
    lines = text.strip().splitlines()
    assert lines[0].split(',') == COLS, f'ヘッダ不一致: {lines[0]}'
    return [[float(v) for v in ln.split(',')] for ln in lines[1:]]


def run_cpp(vx, vy):
    exe = None
    for cand in (WS / 'build' / 'roboone_walk_core' / 'walk_dump',
                 WS / 'install' / 'roboone_walk_core' / 'lib' /
                 'roboone_walk_core' / 'walk_dump'):
        if cand.exists():
            exe = cand
            break
    if exe is None:
        return None
    out = subprocess.run([str(exe), str(vx), str(vy)], capture_output=True,
                         text=True, check=True)
    return parse_csv(out.stdout)


def run_js(vx, vy):
    js = WS / 'src' / 'roboone_motion' / 'roboone_motion' / 'viz' / 'walkcore.js'
    if not js.exists():
        return None
    try:
        out = subprocess.run(['node', str(js), str(vx), str(vy)],
                             capture_output=True, text=True, check=True)
    except FileNotFoundError:
        return None
    return parse_csv(out.stdout)


def compare(name, ref, got):
    if got is None:
        print(f'  {name}: スキップ (未ビルド or node なし)')
        return True
    assert len(ref) == len(got), f'{name}: 行数 {len(ref)} vs {len(got)}'
    worst = {}
    for a, b in zip(ref, got):
        for c, va, vb in zip(COLS, a, b):
            d = abs(va - vb)
            if d > worst.get(c, (0.0,))[0]:
                worst[c] = (d, a[0])
    bad = {c: w for c, w in worst.items() if w[0] > TOL}
    wmax = max((w[0] for w in worst.values()), default=0.0)
    if bad:
        print(f'  {name}: NG  最大誤差 {wmax:.3e}')
        for c, (d, t) in sorted(bad.items(), key=lambda kv: -kv[1][0])[:5]:
            print(f'    {c}: {d:.3e} @ t={t:.3f}')
        return False
    print(f'  {name}: OK  最大誤差 {wmax:.3e}')
    return True


def main():
    ok = True
    for vx, vy in CASES:
        print(f'指令 ({vx:+.2f}, {vy:+.2f}):')
        ref = run_python(vx, vy)
        assert all(math.isfinite(v) for row in ref for v in row)
        ok &= compare('C++', ref, run_cpp(vx, vy))
        ok &= compare('JS ', ref, run_js(vx, vy))
    print('照合: ' + ('全て一致' if ok else '不一致あり'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
