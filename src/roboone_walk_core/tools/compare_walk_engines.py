#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""歩行計画の Python / C++ / JS 3 実装の数値照合。

Python 版 (roboone_walk_ref) が仕様の原本。C++ 版と JS 版が同じ指令プロファイルで
同じ軌道を出すことを、tick ごとの最大絶対誤差で確認する。

    動歩行 walk_core     C++ walk_dump         / JS roboone_viz/walkcore.js
    静歩行 static_walk   C++ static_walk_dump  / JS roboone_viz/staticwalk.js

使い方 (ws ルートから):
    python3 src/roboone_walk_core/tools/compare_walk_engines.py                   # 両方
    python3 src/roboone_walk_core/tools/compare_walk_engines.py --engine static   # 静歩行だけ

libm と V8 の exp() の最終ビット差が e^{ωT} で増幅される分を見込み、
許容誤差は 1e-6 m (0.001 mm) とする。
"""

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys

WS = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WS / 'src' / 'roboone_walk_ref'))

from roboone_walk_ref.static_walk import StaticGaitParams, StaticWalkEngine  # noqa: E402
from roboone_walk_ref.walk_core import GaitParams, WalkEngine  # noqa: E402

DT = 0.005
TOL = 1e-6
CASES = [(0.10, 0.0), (-0.10, 0.0), (0.0, 0.06), (0.0, -0.06),
         (0.08, 0.05), (0.15, 0.08)]
# 両足支持 (ds_time, vx, vy)。3 実装とも ds_time を持つ (JS は 2026-09-19 に移植)
DS_T_END = 10.0
DS_CASES = [(0.2, 0.10, 0.0), (0.4, 0.10, 0.0), (0.4, -0.10, 0.0), (0.4, 0.0, 0.04),
            (0.4, 0.0, -0.04), (0.4, 0.08, 0.025)]
COLS = ['t', 'st', 'ph', 'sup', 'vx', 'vy', 'xix', 'xiy', 'comx', 'comy',
        'zx', 'zy', 'lfx', 'lfy', 'lfz', 'rfx', 'rfy', 'rfz']
# 5, 6 は静歩行だけが使う (C++ の State 列挙・record.py と同じ番号)
STATE_CODE = {'IDLE': 0, 'START': 1, 'STEP': 2, 'STOP': 3, 'ESTOP': 4,
              'SHIFT': 5, 'SWING': 6}

# 静歩行: (vx, vy, 既定値から変える設定)。1 歩に約 2 s かかるので長めに流す
STATIC_T_WALK = 9.5
STATIC_T_END = 20.0
STATIC_CASES = [
    (0.10, 0.0, {}), (-0.10, 0.0, {}), (0.0, 0.04, {}), (0.0, -0.04, {}),
    (0.08, 0.025, {}), (0.15, 0.08, {}),
    (0.10, -0.03, {'com_offset_y': 0.012, 'swing_height': 0.03, 'zmp_tol': 0.008}),
    (-0.06, 0.03, {'com_offset_y': -0.01, 't_swing': 0.5, 'foot_spacing': 0.15}),
]


def run_python(vx, vy, t_walk=4.5, t_end=8.0, engine=None):
    e = engine or WalkEngine()
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


def _find_exe(name):
    for cand in (WS / 'build' / 'roboone_walk_core' / name,
                 WS / 'install' / 'roboone_walk_core' / 'lib' / 'roboone_walk_core' / name):
        if cand.exists():
            return cand
    return None


def run_cpp(vx, vy, exe_name='walk_dump', extra=()):
    exe = _find_exe(exe_name)
    if exe is None:
        return None
    out = subprocess.run([str(exe), str(vx), str(vy), *map(str, extra)],
                         capture_output=True, text=True, check=True)
    return parse_csv(out.stdout)


def run_js(vx, vy, js_name='walkcore.js', extra=()):
    js = WS / 'src' / 'roboone_viz' / 'roboone_viz' / js_name
    if not js.exists():
        return None
    try:
        out = subprocess.run(['node', str(js), str(vx), str(vy), *map(str, extra)],
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


def check_dynamic():
    ok = True
    print('=== 動歩行 (walk_core) ===')
    for vx, vy in CASES:
        print(f'指令 ({vx:+.2f}, {vy:+.2f}):')
        ref = run_python(vx, vy)
        assert all(math.isfinite(v) for row in ref for v in row)
        ok &= compare('C++', ref, run_cpp(vx, vy))
        ok &= compare('JS ', ref, run_js(vx, vy))
    print('--- 両足支持 (ds_time > 0) ---')
    for ds, vx, vy in DS_CASES:
        print(f'ds_time={ds} 指令 ({vx:+.2f}, {vy:+.2f}):')
        ref = run_python(vx, vy, t_end=DS_T_END, engine=WalkEngine(GaitParams(ds_time=ds)))
        assert all(math.isfinite(v) for row in ref for v in row)
        extra = (4.5, DS_T_END, DT, f'ds_time={ds}')
        ok &= compare('C++', ref, run_cpp(vx, vy, extra=extra))
        ok &= compare('JS ', ref, run_js(vx, vy, extra=extra))
    return ok


def check_static_defaults():
    """既定値 (params.py) が C++ (StaticGaitParams) と JS (staticDefaultParams) と同じか。

    足裏の寸法のように軌道に出ない値は、軌道の照合では食い違いに気付けない。
    """
    ref = {k: (list(v) if isinstance(v, (list, tuple)) else [v])
           for k, v in StaticGaitParams().to_dict().items()}
    ok = True
    exe = _find_exe('static_walk_dump')
    got = {}
    if exe is not None:
        out = subprocess.run([str(exe), '--params'], capture_output=True, text=True, check=True)
        got['C++'] = {k: [float(x) for x in v.split(',')]
                      for k, v in (line.split('=', 1) for line in out.stdout.split())}
    js = WS / 'src' / 'roboone_viz' / 'roboone_viz' / 'staticwalk.js'
    try:
        out = subprocess.run(
            ['node', '-e', f'console.log(JSON.stringify(require({json.dumps(str(js))})'
                           '.staticDefaultParams()))'],
            capture_output=True, text=True, check=True)
        got['JS '] = {k: (v if isinstance(v, list) else [v])
                      for k, v in json.loads(out.stdout).items()}
    except FileNotFoundError:
        pass
    for name in ('C++', 'JS '):
        if name not in got:
            print(f'  既定値 {name}: スキップ (未ビルド or node なし)')
            continue
        bad = sorted(k for k in set(ref) | set(got[name])
                     if k not in ref or k not in got[name] or ref[k] != got[name][k])
        if bad:
            ok = False
            print(f'  既定値 {name}: NG  ' +
                  ', '.join(f'{k} (原本 {ref.get(k)} / {got[name].get(k)})' for k in bad))
        else:
            print(f'  既定値 {name}: OK  ({len(ref)} 項目)')
    return ok


def check_static():
    ok = True
    print('=== 静歩行 (static_walk) ===')
    ok &= check_static_defaults()
    for vx, vy, over in STATIC_CASES:
        print(f'指令 ({vx:+.2f}, {vy:+.2f})' + (f' {over}' if over else '') + ':')
        eng = StaticWalkEngine(StaticGaitParams(**over))
        ref = run_python(vx, vy, STATIC_T_WALK, STATIC_T_END, eng)
        assert all(math.isfinite(v) for row in ref for v in row)
        prof = (STATIC_T_WALK, STATIC_T_END, DT)
        ok &= compare('C++', ref, run_cpp(
            vx, vy, 'static_walk_dump', (*prof, *(f'{k}={v}' for k, v in over.items()))))
        ok &= compare('JS ', ref, run_js(
            vx, vy, 'staticwalk.js', (*prof, json.dumps(over))))
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(description='歩行計画の 3 実装の数値照合')
    ap.add_argument('--engine', choices=('all', 'dynamic', 'static'), default='all')
    args = ap.parse_args(argv)
    ok = True
    if args.engine in ('all', 'dynamic'):
        ok &= check_dynamic()
    if args.engine in ('all', 'static'):
        ok &= check_static()
    print('照合: ' + ('全て一致' if ok else '不一致あり'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
