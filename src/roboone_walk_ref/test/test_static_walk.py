# -*- coding: utf-8 -*-
"""static_walk (静歩行) の単体テスト。

静歩行の定義そのもの — **全時刻で ZMP と重心が支持多角形の中にある** — を、
前後・左右・斜め・指令の切り替え・任意の時刻での停止について確かめる。
あわせて、支持足が動かないこと、歩幅が振り出しの開始で固定されること、
停止で足が揃って重心が中点へ戻ることを見る。
C++ / JS との数値一致は roboone_walk_core/tools/compare_walk_engines.py。
"""

import math
from pathlib import Path

import pytest

from roboone_walk_ref.static_walk import (check_static_gait, SHIFT, StaticGaitParams,
                                          StaticWalkEngine, support_margin, SWING)
from roboone_walk_ref.walk_core import IDLE, LEFT, RIGHT, STOP

DT = 0.005
YAML = Path(__file__).resolve().parents[1] / 'config' / 'static_gait.yaml'


def run(cmd_fn, t_end, params=None):
    """cmd_fn(t) -> (vx, vy) を t_end 秒流して (エンジン, 出力列) を返す。"""
    e = StaticWalkEngine(params)
    outs = [e.update(*cmd_fn(i * DT), DT) for i in range(int(round(t_end / DT)))]
    return e, outs


def stop_after(t_stop, vx, vy):
    return lambda t: (vx, vy) if 0.5 <= t < t_stop else (0.0, 0.0)


def stick(t):
    """指令を動かし続ける (前進 -> 斜め -> 後進 -> 横 -> 停止)。"""
    if t < 0.5 or t >= 24.0:
        return (0.0, 0.0)
    return (0.10 * math.sin(0.37 * t), 0.04 * math.cos(0.23 * t))


PROFILES = {
    'fwd': stop_after(9.0, 0.10, 0.0),
    'back': stop_after(9.0, -0.10, 0.0),
    'left': stop_after(9.0, 0.0, 0.04),
    'right': stop_after(9.0, 0.0, -0.04),
    'diag': stop_after(9.0, 0.08, -0.025),
    'diag_back': stop_after(9.0, -0.08, 0.025),
    'rev': lambda t: (0.10, 0.0) if 0.5 <= t < 7.0 else ((-0.10, 0.0) if t < 14.0 else (0.0, 0.0)),
    'stick': stick,
}


# ------------------------------------------------------------------ 設定
def test_yaml_matches_defaults():
    """static_gait.yaml と params.py の既定値が同じ (C++ の既定はこれを写す)。"""
    p = StaticGaitParams.from_yaml(str(YAML))
    assert p == StaticGaitParams()
    assert check_static_gait(p)['errors'] == []


def test_yaml_rejects_unknown_key():
    with pytest.raises(KeyError):
        StaticGaitParams.from_dict({'t_step': 0.6})


def test_check_catches_bad_settings():
    # 降下が間に合わない: 0.55 x 0.2s で 54mm は 0.1 m/s では降りきれない
    r = check_static_gait(StaticGaitParams(t_swing=0.2, swing_height=0.05, td_speed_max=0.10))
    assert not r['lands'] and r['errors']
    # 重心を足裏の縁まで外へずらす
    r = check_static_gait(StaticGaitParams(com_offset_y=0.035))
    assert r['errors']
    # 既定では片足支持の余裕が足裏の半幅そのまま
    r = check_static_gait(StaticGaitParams())
    assert r['margin_single'] == pytest.approx((0.059, 0.037))
    assert r['t_shift_step'] == pytest.approx(1.466, abs=1e-3)


# ------------------------------------------------------------ 静的な安定
@pytest.mark.parametrize('name', sorted(PROFILES))
@pytest.mark.parametrize('offset', [0.0, 0.01, -0.01])
def test_zmp_and_com_inside_support(name, offset):
    """全時刻で ZMP と重心が支持多角形の中 (余裕 = 足裏半幅 - |offset| - zmp_tol 以上)。"""
    p = StaticGaitParams(com_offset_y=offset)
    need = p.sole_width / 2.0 - abs(offset) - p.zmp_tol - 1e-9
    _, outs = run(PROFILES[name], 30.0, p)
    worst = min(support_margin(o, p) for o in outs)
    assert worst >= need, f'{name}: ZMP の余裕 {worst * 1000:.2f}mm'
    worst_c = min(support_margin(o, p, o.com) for o in outs)
    assert worst_c >= p.sole_width / 2.0 - abs(offset) - 1e-9


@pytest.mark.parametrize('name', sorted(PROFILES))
def test_zmp_close_to_com(name):
    """重心の加速度で ZMP が重心の真下から zmp_tol 以上ずれない。"""
    p = StaticGaitParams()
    _, outs = run(PROFILES[name], 30.0)
    worst = max(math.hypot(o.zmp[0] - o.com[0], o.zmp[1] - o.com[1]) for o in outs)
    assert worst <= p.zmp_tol + 1e-9
    assert worst > 0.9 * p.zmp_tol       # 上限いっぱいまで使って急いでいる


@pytest.mark.parametrize('name', sorted(PROFILES))
def test_com_still_over_support_while_swinging(name):
    p = StaticGaitParams(com_offset_y=0.008)
    _, outs = run(PROFILES[name], 30.0, p)
    n = 0
    for o in outs:
        if o.state != SWING:
            continue
        n += 1
        f = o.left_foot if o.support == LEFT else o.right_foot
        assert o.com[0] == pytest.approx(f[0], abs=1e-12)
        assert o.com[1] == pytest.approx(f[1] + o.support * p.com_offset_y, abs=1e-12)
        assert o.xi == o.com and o.zmp == o.com
    assert n > 0


@pytest.mark.parametrize('name', sorted(PROFILES))
def test_only_swing_foot_moves(name):
    """足が動くのは SWING 中の遊脚だけ。支持足と、両足支持の間の足は動かない。

    着地の周期は出力の状態がもう次 (SHIFT / STOP) になっているが、遊脚は振り出しの
    最後のぶんだけ動くので、前の周期の状態で見る。
    """
    _, outs = run(PROFILES[name], 30.0)
    for a, b in zip(outs, outs[1:]):
        swing = b if b.state == SWING else a
        for side, fa, fb in ((LEFT, a.left_foot, b.left_foot),
                             (RIGHT, a.right_foot, b.right_foot)):
            moved = fa[:2] != fb[:2]
            if moved:
                assert swing.state == SWING and swing.support == -side, (b.t, b.state, side)


@pytest.mark.parametrize('name', sorted(PROFILES))
def test_trajectories_are_continuous(name):
    """重心・足先に飛びが無い。z は着地で overdrive ぶん (押し込みを抜く) だけ戻る。"""
    p = StaticGaitParams()
    _, outs = run(PROFILES[name], 30.0)
    v_foot = 3.0 * max(p.v_max) * p.stride_time / p.t_swing + p.foot_spacing / p.t_swing * 2.0
    for a, b in zip(outs, outs[1:]):
        assert math.hypot(b.com[0] - a.com[0], b.com[1] - a.com[1]) < 0.5 * DT
        for fa, fb in ((a.left_foot, b.left_foot), (a.right_foot, b.right_foot)):
            assert math.hypot(fb[0] - fa[0], fb[1] - fa[1]) < v_foot * DT
            assert abs(fb[2] - fa[2]) <= p.td_overdrive + 1e-9


# ------------------------------------------------------------ 歩幅と着地
def test_stride_latched_at_swing_start():
    """着地点は振り出しの開始で決まり、遊脚中に指令が変わっても動かない。"""
    p = StaticGaitParams()

    def cmd(t):
        return (0.10, 0.0) if int(t * 3) % 2 == 0 else (-0.10, 0.03)
    e, outs = run(cmd, 25.0)
    for a, b in zip(outs, outs[1:]):
        if a.state == SWING and b.state == SWING:
            assert a.p_land == b.p_land
    walked = [r for r in e.steps if r.mode == 'walk']
    assert len(walked) >= 5
    for r in walked:
        swing = -r.support
        assert r.p_land[0] == pytest.approx(r.p_support[0] + r.v[0] * p.stride_time, abs=1e-12)
        assert r.p_land[1] == pytest.approx(
            r.p_support[1] + swing * p.foot_spacing + r.v[1] * p.stride_time, abs=1e-12)


def test_swing_lands_and_height_profile():
    p = StaticGaitParams()
    _, outs = run(PROFILES['fwd'], 20.0)
    zs = [max(o.left_foot[2], o.right_foot[2]) for o in outs]
    assert max(zs) == pytest.approx(p.swing_height, abs=1e-3)
    assert min(zs) >= -p.td_overdrive - 1e-9
    # どの振り出しも、終わる前に床 (z <= 0) へ届いている
    swinging = False
    touched = False
    for o in outs:
        z = max(o.left_foot[2], o.right_foot[2]) if o.state == SWING else None
        if o.state == SWING and not swinging:
            swinging, touched = True, False
        if o.state == SWING and o.phase > 0.0 and z <= 0.0:
            touched = True
        if o.state != SWING and swinging:
            assert touched
            swinging = False


def test_feet_never_overlap():
    p = StaticGaitParams()
    for name, fn in PROFILES.items():
        _, outs = run(fn, 30.0)
        for o in outs:
            gap = o.left_foot[1] - o.right_foot[1]
            assert gap > p.sole_width, f'{name} t={o.t} gap={gap}'


def test_lateral_leads_with_travel_side_foot():
    e, _ = run(stop_after(3.0, 0.0, 0.04), 4.0)
    assert e.steps[0].support == RIGHT     # 左へ → 右足支持で左足から
    e, _ = run(stop_after(3.0, 0.0, -0.04), 4.0)
    assert e.steps[0].support == LEFT      # 右へ → 左足支持で右足から


@pytest.mark.parametrize('vx,vy,ex,ey', [
    (0.10, 0.0, 1, 0), (-0.10, 0.0, -1, 0),
    (0.0, 0.04, 0, 1), (0.0, -0.04, 0, -1),
    (0.08, 0.025, 1, 1), (-0.08, -0.025, -1, -1),
])
def test_net_displacement_direction(vx, vy, ex, ey):
    e, outs = run(stop_after(12.0, vx, vy), 25.0)
    o = outs[-1]
    assert o.state == IDLE
    mid = [(o.left_foot[0] + o.right_foot[0]) / 2, (o.left_foot[1] + o.right_foot[1]) / 2]
    for m, want in zip(mid, (ex, ey)):
        if want:
            assert m * want > 0.02
        else:
            assert abs(m) < 1e-9


def test_forward_speed_matches_check():
    """定常前進の速さが check_static_gait の見積もりどおり。"""
    p = StaticGaitParams()
    r = check_static_gait(p)
    e, outs = run(lambda t: (0.10, 0.0), 60.0)
    walked = [s for s in e.steps if s.mode == 'walk'][4:]
    dt = walked[-1].t_start - walked[0].t_start
    dx = walked[-1].p_support[0] - walked[0].p_support[0]
    assert dx / dt == pytest.approx(r['v_fwd_real'], rel=0.01)


# ------------------------------------------------------------------ 停止
def _assert_stopped(o, p):
    assert o.state == IDLE
    lf, rf = o.left_foot, o.right_foot
    assert lf[2] == 0.0 and rf[2] == 0.0
    assert lf[0] == pytest.approx(rf[0], abs=1e-9)
    assert lf[1] - rf[1] == pytest.approx(p.foot_spacing, abs=1e-9)
    assert o.com[0] == pytest.approx((lf[0] + rf[0]) / 2, abs=1e-12)
    assert o.com[1] == pytest.approx((lf[1] + rf[1]) / 2, abs=1e-12)


@pytest.mark.parametrize('cmd', [(0.10, 0.0), (0.08, -0.025), (0.0, 0.04)])
@pytest.mark.parametrize('t_stop', [0.52 + 0.37 * k for k in range(24)])
def test_stops_from_any_time(cmd, t_stop):
    """どの位相で指令を離しても、足を揃えて中点に立つ。

    整形後の速度は a_max で落ちるので、離してすぐは小さな歩を踏むことがある。
    速度が v_stop_eps を下回ったあとに始まる歩は、足を揃える 1 歩だけ。
    """
    p = StaticGaitParams()
    e, outs = run(stop_after(t_stop, *cmd), t_stop + 12.0)
    _assert_stopped(outs[-1], p)
    t_quiet = next(o.t for o in outs
                   if o.t > t_stop and math.hypot(*o.v) < p.v_stop_eps)
    after = [s for s in e.steps if s.t_start >= t_quiet]
    assert len(after) <= 1
    assert all(s.mode == 'stop' for s in after)


def test_immediate_release_does_not_step():
    """一瞬だけ倒して離す: 重心を寄せたあと振り出さずに戻る。"""
    p = StaticGaitParams()
    e, outs = run(lambda t: (0.10, 0.0) if 0.5 <= t < 0.6 else (0.0, 0.0), 8.0)
    _assert_stopped(outs[-1], p)
    assert all(o.state in (IDLE, SHIFT, STOP, SWING) for o in outs)
    assert outs[-1].left_foot[:2] == (0.0, p.foot_spacing / 2)


def test_resume_during_stop_finishes_first():
    """STOP の途中で指令が戻っても、中点へ戻り切ってから歩き直す。"""
    p = StaticGaitParams()
    e = StaticWalkEngine(p)
    seen_stop = False
    states = []
    for i in range(int(40.0 / DT)):
        t = i * DT
        cmd = (0.10, 0.0) if (0.5 <= t < 5.0 or (seen_stop and t < 30.0)) else (0.0, 0.0)
        o = e.update(*cmd, DT)
        if o.state == STOP:
            seen_stop = True
        states.append(o.state)
    # STOP の直後は必ず IDLE を 1 周期以上挟む
    for a, b in zip(states, states[1:]):
        if a == STOP and b != STOP:
            assert b == IDLE
    _assert_stopped(o, p)
    assert sum(1 for s in e.steps if s.mode == 'walk') > 4


# ---------------------------------------------------------------- 決定性ほか
def test_deterministic():
    _, a = run(stick, 12.0)
    _, b = run(stick, 12.0)
    assert a == b


def test_command_shaping_saturates():
    p = StaticGaitParams()
    _, outs = run(lambda t: (10.0, 10.0), 5.0)
    for o in outs:
        s = math.hypot(o.v[0] / p.v_max[0], o.v[1] / p.v_max[1])
        assert s <= 1.0 + 1e-6


def test_estop_freezes():
    e = StaticWalkEngine()
    for _ in range(int(3.0 / DT)):
        e.update(0.1, 0.0, DT)
    o1 = e.update(0.1, 0.0, DT, estop=True)
    o2 = e.update(0.1, 0.0, DT, estop=False)   # ラッチ: 解除は reset (home 技) から
    assert o1.state == 'ESTOP' and o2.state == 'ESTOP'
    assert o1.left_foot == o2.left_foot and o1.com == o2.com
    e.reset()
    assert e.update(0.0, 0.0, DT).state == IDLE
