# -*- coding: utf-8 -*-
"""walk_core の単体テスト。

docs/ros2_walk_implementation.pdf §9 の「ROS なしの単体テスト」に対応する:
  * 生成した p(t) と xC(t) が ẍC = ω²(xC − p) を満たす (数値微分で残差を見る)
  * 各周期で ZMP 参照が支持足 (静止時は支持多角形) の上にある
  * START → STEP → STOP の一連で ξ が発散せず中点に収束する
  * 同じ入力列で同じ出力列が出る (決定性)
IK の往復テストは roboone_kinematics 側にある。
"""

import math

import pytest

from roboone_walk_ref.walk_core import (GaitParams, IDLE, STEP, WalkEngine)

DT = 0.005


def run(cmd_fn, t_end, engine=None):
    """cmd_fn(t) -> (vx, vy) を t_end 秒流して出力列を返す。"""
    e = engine or WalkEngine()
    outs = []
    for i in range(int(round(t_end / DT))):
        outs.append(e.update(*cmd_fn(i * DT), DT))
    return e, outs


def stop_after(t_stop, vx, vy):
    return lambda t: (vx, vy) if t < t_stop else (0.0, 0.0)


# ---------------------------------------------------------------- 力学の整合
def test_com_follows_lipm():
    """重心が ẍC = ω²(xC − p) を満たすこと (歩の内部、ZMP 一定の区間で確認)。"""
    p = GaitParams()
    w2 = p.omega ** 2
    _, outs = run(stop_after(3.0, 0.10, 0.0), 5.0)
    checked = 0
    for k in range(1, len(outs) - 1):
        a, b, c = outs[k - 1], outs[k], outs[k + 1]
        # ZMP が動いた周期 (歩の境界) は除く
        if not (a.state == b.state == c.state == STEP):
            continue
        if a.zmp != b.zmp or b.zmp != c.zmp:
            continue
        for ax in (0, 1):
            acc = (c.com[ax] - 2 * b.com[ax] + a.com[ax]) / DT / DT
            ref = w2 * (b.com[ax] - b.zmp[ax])
            # オイラー積分 (文書どおり) なので O(ω·dt) ≈ 4% の残差は許す
            assert acc == pytest.approx(ref, abs=max(0.10 * abs(ref), 0.02))
        checked += 1
    assert checked > 300


def test_xi_closed_form():
    """ξ が ξ̇ = ω(ξ − p) の閉形式に乗っていること。"""
    p = GaitParams()
    _, outs = run(stop_after(3.0, 0.10, 0.03), 5.0)
    for k in range(len(outs) - 1):
        a, b = outs[k], outs[k + 1]
        if not (a.state == b.state == STEP) or a.zmp != b.zmp:
            continue
        e = math.exp(p.omega * DT)
        for ax in (0, 1):
            pred = a.zmp[ax] + (a.xi[ax] - a.zmp[ax]) * e
            # 歩の最終周期は境界処理で t_local が T に丸められるので緩めに見る
            assert b.xi[ax] == pytest.approx(pred, abs=5e-3)


def test_zmp_stays_under_feet():
    """ZMP 参照が常に接地している足の凸包 (線分近似 + 足裏半長) の中にあること。"""
    half_foot = 0.05   # 足裏の半長のつもりの余裕
    _, outs = run(stop_after(3.0, 0.12, 0.04), 6.0)
    for o in outs:
        lf, rf = o.left_foot, o.right_foot
        grounded = [f for f in (lf, rf) if f[2] < 1e-9]
        d = min(_dist_to_segment(o.zmp, grounded[0], grounded[-1]), 1e9)
        assert d < half_foot, f't={o.t} zmp={o.zmp} feet={grounded}'


def _dist_to_segment(pt, a, b):
    abx, aby = b[0] - a[0], b[1] - a[1]
    den = abx * abx + aby * aby
    dot = (pt[0] - a[0]) * abx + (pt[1] - a[1]) * aby
    u = 0.0 if den == 0 else max(0.0, min(1.0, dot / den))
    px, py = a[0] + u * abx, a[1] + u * aby
    return math.hypot(pt[0] - px, pt[1] - py)


# ------------------------------------------------------------ 収束と停止姿勢
@pytest.mark.parametrize('vx,vy', [
    (0.10, 0.0), (0.15, 0.0), (-0.10, 0.0),
    (0.0, 0.06), (0.0, -0.06), (0.08, 0.05), (-0.08, -0.05),
])
def test_start_walk_stop_converges(vx, vy):
    """全方向で START → STEP → STOP が完走し、足が揃って ξ が中点に収束すること。"""
    p = GaitParams()
    e, outs = run(stop_after(4.0, vx, vy), 8.0)
    o = outs[-1]
    assert o.state == IDLE
    lf, rf = o.left_foot, o.right_foot
    # 足が揃う: y 間隔は W、x はそろう
    assert lf[1] - rf[1] == pytest.approx(p.foot_spacing, abs=1e-3)
    assert lf[0] - rf[0] == pytest.approx(0.0, abs=1e-3)
    # ξ と重心が両足の中点にある
    mid = [(lf[0] + rf[0]) / 2, (lf[1] + rf[1]) / 2]
    for ax in (0, 1):
        assert o.xi[ax] == pytest.approx(mid[ax], abs=2e-3)
        assert o.com[ax] == pytest.approx(mid[ax], abs=2e-3)
    # 発散していない
    for oo in outs:
        assert all(math.isfinite(v) for v in (*oo.xi, *oo.com, *oo.zmp))
        assert abs(oo.xi[0]) < 2.0 and abs(oo.xi[1]) < 2.0


@pytest.mark.parametrize('vx,vy,ex,ey', [
    (0.10, 0.0, 1, 0), (-0.10, 0.0, -1, 0),
    (0.0, 0.06, 0, 1), (0.0, -0.06, 0, -1),
    (0.08, 0.05, 1, 1), (0.08, -0.05, 1, -1),
])
def test_net_displacement_direction(vx, vy, ex, ey):
    """正味の移動が指令の方向を向くこと (回転なしの平行移動)。"""
    e, outs = run(stop_after(4.0, vx, vy), 8.0)
    o = outs[-1]
    mid = [(o.left_foot[0] + o.right_foot[0]) / 2,
           (o.left_foot[1] + o.right_foot[1]) / 2]
    if ex:
        assert mid[0] * ex > 0.15
    else:
        assert abs(mid[0]) < 0.02
    if ey:
        assert mid[1] * ey > 0.08
    else:
        assert abs(mid[1]) < 0.02


def test_feet_never_cross():
    """左足は常に右足より左 (y_L > y_R)。脚同士の干渉なし。"""
    for cmd in [(0.12, 0.0), (0.0, 0.08), (0.0, -0.08), (0.10, 0.06), (-0.10, -0.06)]:
        _, outs = run(stop_after(4.0, *cmd), 8.0)
        for o in outs:
            gap = o.left_foot[1] - o.right_foot[1]
            assert gap > 0.03, f'cmd={cmd} t={o.t} gap={gap}'


def test_lateral_leads_with_travel_side_foot():
    """横移動は進行方向側の足から踏み出すこと。"""
    e, _ = run(stop_after(2.0, 0.0, 0.06), 3.0)
    assert e.steps[0].support == -1     # 左へ → 右足支持で左足から
    e, _ = run(stop_after(2.0, 0.0, -0.06), 3.0)
    assert e.steps[0].support == +1     # 右へ → 左足支持で右足から


# ---------------------------------------------------------------- 決定性ほか
def test_deterministic():
    """同じ入力列で同じ出力列 (文書 §1.3)。"""
    def cmd(t):
        return (0.08 * math.sin(t), 0.04 * math.cos(1.3 * t))
    _, a = run(cmd, 6.0)
    _, b = run(cmd, 6.0)
    for oa, ob in zip(a, b):
        assert oa == ob


def test_command_shaping_saturates():
    """指令の飽和と楕円制限 (式 1, 2 読み替え)。"""
    p = GaitParams()
    e, outs = run(lambda t: (10.0, 10.0), 3.0)
    for o in outs:
        assert abs(o.v[0]) <= p.v_max[0] + 1e-9
        assert abs(o.v[1]) <= p.v_max[1] + 1e-9
        s = math.hypot(o.v[0] / p.v_max[0], o.v[1] / p.v_max[1])
        assert s <= 1.0 + 1e-6


def test_landing_within_clamp():
    """確定した着地点が名目からのクランプ域に収まること (式 11)。"""
    p = GaitParams()

    def cmd(t):
        # 荒い指令変化でもクランプ内
        return (0.15, 0.08) if int(t * 2) % 2 == 0 else (-0.15, -0.08)
    e, _ = run(cmd, 8.0)
    walked = [r for r in e.steps if r.p_land is not None]
    assert len(walked) > 5
    for r in walked:
        assert abs(r.p_land[0] - r.p_nom[0]) <= p.step_clamp_x + 1e-9
        dy = r.p_land[1] - r.p_nom[1]
        assert -p.step_clamp_out - 1e-9 <= dy <= p.step_clamp_out + 1e-9


def test_swing_foot_height_profile():
    """遊脚が持ち上がって頂点 h_sw に達し、着地で 0 に戻ること。"""
    p = GaitParams()
    _, outs = run(stop_after(3.0, 0.10, 0.0), 5.0)
    zmax = max(max(o.left_foot[2], o.right_foot[2]) for o in outs)
    assert zmax == pytest.approx(p.swing_height, abs=2e-3)
    for o in outs:
        z = max(o.left_foot[2], o.right_foot[2])
        assert -p.td_overdrive - 1e-6 <= z <= p.swing_height + 1e-6


def test_estop_freezes():
    e = WalkEngine()
    for i in range(200):
        e.update(0.1, 0.0, DT)
    o1 = e.update(0.1, 0.0, DT, estop=True)
    o2 = e.update(0.1, 0.0, DT, estop=False)   # ラッチ: 解除は reset (home 技) から
    assert o1.state == 'ESTOP' and o2.state == 'ESTOP'
    assert o1.left_foot == o2.left_foot and o1.xi == o2.xi


# ------------------------------------------------------------ 両足支持 (ds_time)
def test_ds_default_is_off():
    """ds_time の既定は 0 で、両足支持の出力は立たない (従来と同じ)。"""
    assert GaitParams().ds_time == 0.0
    _, outs = run(stop_after(3.0, 0.10, 0.0), 6.0)
    assert not any(o.double_support for o in outs)


def test_ds_dcm_follows_moving_zmp():
    """両足支持で ZMP が動いている間も、ξ と重心が LIPM の式に乗っていること。

    ξ が ξ̇ = ω(ξ − p) を満たし、重心が ẋC = ω(ξ − xC) で進めば ẍC = ω²(xC − p) になる。
    ξ は閉形式なので中点則で細かく見る。重心の加速度を二階差分で見ないのは、
    オイラー積分の誤差 (ω²·dt·|ṗ| ≈ 0.04 m/s²) が、0 を横切る加速度に比べて大きいから。
    """
    p = GaitParams(ds_time=0.4)
    w = p.omega
    _, outs = run(stop_after(3.0, 0.08, 0.02), 6.0, WalkEngine(p))
    checked = 0
    for k in range(len(outs) - 1):
        a, b = outs[k], outs[k + 1]
        # 入った周期は、前の区間の積分で進んだ値なので除く
        if not (a.double_support and b.double_support and a.ds_elapsed > 0.0):
            continue
        for ax in (0, 1):
            dxi = (b.xi[ax] - a.xi[ax]) / DT
            mid = w * ((a.xi[ax] + b.xi[ax]) / 2 - (a.zmp[ax] + b.zmp[ax]) / 2)
            assert dxi == pytest.approx(mid, abs=2e-3)
            assert b.com[ax] == pytest.approx(
                a.com[ax] + w * (b.xi[ax] - a.com[ax]) * DT, abs=1e-12)
        checked += 1
    assert checked > 200


@pytest.mark.parametrize('ds,vx,vy', [
    (0.2, 0.10, 0.0), (0.4, 0.10, 0.0), (0.4, -0.10, 0.0), (0.4, 0.0, 0.04),
    (0.4, 0.0, -0.04), (0.4, 0.08, 0.025), (0.4, 0.0, 0.0),
])
def test_ds_start_walk_stop_converges(ds, vx, vy):
    """両足支持ありでも歩き出して止まり、足が揃って ξ が中点に来ること。"""
    p = GaitParams(ds_time=ds)
    e, outs = run(stop_after(4.5, vx, vy), 12.0, WalkEngine(p))
    o = outs[-1]
    assert o.state == IDLE
    lf, rf = o.left_foot, o.right_foot
    assert lf[1] - rf[1] == pytest.approx(p.foot_spacing, abs=1e-3)
    assert lf[0] - rf[0] == pytest.approx(0.0, abs=1e-3)
    mid = ((lf[0] + rf[0]) / 2, (lf[1] + rf[1]) / 2)
    assert o.xi[0] == pytest.approx(mid[0], abs=2e-3)
    assert o.xi[1] == pytest.approx(mid[1], abs=2e-3)
    assert max(abs(v) for oo in outs for v in oo.xi) < 1.0
    # 指令どおりに歩いた区間では着地点のクランプに当たらない (a_max の範囲内)
    assert not any(r.clamped for r in e.steps)


def test_ds_steady_state_repeats():
    """一定の指令で歩き続けると、同じ足の歩の頭の状態 (ZMP の出発点から見た ξ) が繰り返すこと。"""
    p = GaitParams(ds_time=0.4)
    e = WalkEngine(p)
    starts = {1: [], -1: []}
    was_ds = False
    for i in range(int(9.0 / DT)):
        o = e.update(0.0, 0.03, DT)
        if o.double_support and not was_ds and o.state == STEP:
            starts[e.sup].append((o.xi[0] - e.ds_from[0], o.xi[1] - e.ds_from[1]))
        was_ds = o.double_support
    for side in (1, -1):
        s = starts[side]
        assert len(s) >= 3
        # 立ち上がり (a_max で指令が伸びる間) を過ぎた 2 つを比べる
        (ax, ay), (bx, by) = s[-2], s[-1]
        assert ax == pytest.approx(bx, abs=1e-4)
        assert ay == pytest.approx(by, abs=1e-4)


def test_ds_outputs_during_double_support():
    """両足支持の間は support=0・phase=0・両足接地で、ZMP は両足の間を動く。"""
    p = GaitParams(ds_time=0.4)
    _, outs = run(stop_after(3.0, 0.10, 0.03), 8.0, WalkEngine(p))
    runs, cur = [], 0
    for o in outs:
        if o.double_support:
            assert o.support == 0 and o.phase == 0.0
            assert o.left_foot[2] == 0.0 and o.right_foot[2] == 0.0
            assert _dist_to_segment(o.zmp, o.left_foot, o.right_foot) < 1e-9
            # 入った周期は経過 0 (その周期はまだ前の区間の積分で進んでいる)
            assert 0.0 <= o.ds_elapsed < p.ds_time
            cur += 1
        else:
            assert o.ds_elapsed == 0.0
            if cur:
                runs.append(cur)
            cur = 0
    # 両足支持は毎回 Td ちょうど (最後の両足支持も含む)
    n = int(round(p.ds_time / DT))
    assert len(runs) >= 4
    assert all(r == n for r in runs)
