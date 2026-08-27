# -*- coding: utf-8 -*-
"""歩行計画のシナリオを走らせて、可視化用の時系列データに落とす。

エンジンは 200 Hz で回し、記録は 100 Hz に間引く (ファイルサイズ半減、
見た目には十分)。数値は 0.1 mm (4 桁) に丸めて JSON を小さくする。
"""

from dataclasses import dataclass
from typing import Callable, List, Optional

from ..walk_core import GaitParams, WalkEngine

STATE_CODE = {'IDLE': 0, 'START': 1, 'STEP': 2, 'STOP': 3, 'ESTOP': 4}
ENGINE_DT = 0.005
RECORD_EVERY = 2          # 100 Hz で記録


@dataclass
class Scenario:
    sid: str
    label: str
    desc: str
    duration: float
    cmd: Callable[[float], tuple]   # t -> (vx, vy) 生指令


def default_scenarios() -> List[Scenario]:
    """前後・左右・斜め、それとスティックを動かし続ける操縦シナリオ。"""
    return [
        Scenario('fwd', '前進', 'vx=+0.10 m/s を 4 s → 停止', 8.0,
                 lambda t: (0.10, 0.0) if 0.5 <= t < 4.5 else (0.0, 0.0)),
        Scenario('back', '後進', 'vx=−0.10 m/s を 4 s → 停止', 8.0,
                 lambda t: (-0.10, 0.0) if 0.5 <= t < 4.5 else (0.0, 0.0)),
        Scenario('left', '左移動', 'vy=+0.06 m/s (左足から踏み出す)', 8.0,
                 lambda t: (0.0, 0.06) if 0.5 <= t < 4.5 else (0.0, 0.0)),
        Scenario('right', '右移動', 'vy=−0.06 m/s (右足から踏み出す)', 8.0,
                 lambda t: (0.0, -0.06) if 0.5 <= t < 4.5 else (0.0, 0.0)),
        Scenario('diag', '斜め前 (左)', 'vx=+0.08, vy=+0.05 の平行移動。機体は回転しない', 8.0,
                 lambda t: (0.08, 0.05) if 0.5 <= t < 4.5 else (0.0, 0.0)),
        Scenario('stick', 'スティック操縦', '前進 → 斜め右前 → 斜め左前 → 停止 と指令を切り替える', 10.0,
                 _stick_profile),
    ]


def _stick_profile(t: float) -> tuple:
    if t < 0.5:
        return (0.0, 0.0)
    if t < 2.5:
        return (0.08, 0.0)
    if t < 4.5:
        return (0.10, -0.05)
    if t < 6.5:
        return (0.06, 0.05)
    return (0.0, 0.0)


def _r(v: Optional[float], nd: int = 4):
    if v is None:
        return None
    return round(v, nd)


def record_scenario(sc: Scenario, params: Optional[GaitParams] = None) -> dict:
    p = params or GaitParams()
    eng = WalkEngine(p)
    cols = {k: [] for k in (
        't', 'st', 'ph', 'sup', 'stop', 'lock',
        'cx', 'cy',            # 生指令 (ジョイスティック)
        'vx', 'vy',            # 整形後
        'xix', 'xiy', 'comx', 'comy', 'zx', 'zy',
        'lfx', 'lfy', 'lfz', 'rfx', 'rfy', 'rfz',
        'pnx', 'pny', 'plx', 'ply', 'bx', 'by', 'xex', 'xey')}
    boxes = []                 # クランプ域は変化時だけ [frame, xmin,xmax,ymin,ymax]
    last_box = object()
    n = int(round(sc.duration / ENGINE_DT))
    frame = 0
    for i in range(n):
        t = i * ENGINE_DT
        raw = sc.cmd(t)
        o = eng.update(raw[0], raw[1], ENGINE_DT)
        if i % RECORD_EVERY:
            continue
        cols['t'].append(_r(o.t, 3))
        cols['st'].append(STATE_CODE[o.state])
        cols['ph'].append(_r(o.phase, 3))
        cols['sup'].append(o.support)
        cols['stop'].append(1 if o.stopping else 0)
        cols['lock'].append(1 if o.locked else 0)
        cols['cx'].append(_r(raw[0]))
        cols['cy'].append(_r(raw[1]))
        cols['vx'].append(_r(o.v[0]))
        cols['vy'].append(_r(o.v[1]))
        cols['xix'].append(_r(o.xi[0]))
        cols['xiy'].append(_r(o.xi[1]))
        cols['comx'].append(_r(o.com[0]))
        cols['comy'].append(_r(o.com[1]))
        cols['zx'].append(_r(o.zmp[0]))
        cols['zy'].append(_r(o.zmp[1]))
        cols['lfx'].append(_r(o.left_foot[0]))
        cols['lfy'].append(_r(o.left_foot[1]))
        cols['lfz'].append(_r(o.left_foot[2]))
        cols['rfx'].append(_r(o.right_foot[0]))
        cols['rfy'].append(_r(o.right_foot[1]))
        cols['rfz'].append(_r(o.right_foot[2]))
        cols['pnx'].append(_r(o.p_nom[0]) if o.p_nom else None)
        cols['pny'].append(_r(o.p_nom[1]) if o.p_nom else None)
        cols['plx'].append(_r(o.p_land[0]) if o.p_land else None)
        cols['ply'].append(_r(o.p_land[1]) if o.p_land else None)
        cols['bx'].append(_r(o.b_next[0]) if o.b_next else None)
        cols['by'].append(_r(o.b_next[1]) if o.b_next else None)
        cols['xex'].append(_r(o.xi_eos[0]) if o.xi_eos else None)
        cols['xey'].append(_r(o.xi_eos[1]) if o.xi_eos else None)
        box = tuple(_r(v) for v in o.clamp_box) if o.clamp_box else None
        if box != last_box:
            boxes.append([frame] + (list(box) if box else [None]))
            last_box = box
        frame += 1

    steps = []
    for r in eng.steps:
        steps.append({
            'i': r.step_idx, 'mode': r.mode, 'sup': r.support,
            't0': _r(r.t_start, 3), 't1': _r(r.t_end, 3),
            'v': [_r(r.v[0]), _r(r.v[1])],
            'psup': [_r(r.p_support[0]), _r(r.p_support[1])],
            'pnom': [_r(r.p_nom[0]), _r(r.p_nom[1])] if r.p_nom else None,
            'pland': [_r(r.p_land[0]), _r(r.p_land[1])] if r.p_land else None,
            'b': [_r(r.b_next[0], 5), _r(r.b_next[1], 5)] if r.b_next else None,
            'clamped': bool(r.clamped),
        })
    return {
        'id': sc.sid, 'label': sc.label, 'desc': sc.desc,
        'dt': ENGINE_DT * RECORD_EVERY,
        'ticks': cols, 'boxes': boxes, 'steps': steps,
    }


def build_dataset(scenarios: Optional[List[Scenario]] = None,
                  params: Optional[GaitParams] = None) -> dict:
    p = params or GaitParams()
    scs = scenarios or default_scenarios()
    b_ss = 0.10 * p.t_step / (p.e_wt - 1.0)
    return {
        'params': p.to_dict(),
        'derived': {
            'omega': round(p.omega, 4),
            'e_wt': round(p.e_wt, 2),
            'b_x_at_0.10': round(b_ss, 5),
            'b_y_lateral': round(p.foot_spacing / (p.e_wt + 1.0), 5),
        },
        'scenarios': [record_scenario(s, p) for s in scs],
    }
