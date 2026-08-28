# -*- coding: utf-8 -*-
"""膝 4 節リンク (knee_fourbar.py) の受け入れテスト。

``docs/膝4節リンク導出.tex`` §9 の数値検算にあたる。判定基準は依頼の受け入れ条件を
そのまま写したもので、閾値は緩めない。

  1. FK の出力でループが閉じる                       残差 < 1e-9 mm
  2. IK(FK(θ2)) == θ2                               誤差 < 1e-9 deg   ← 枝の誤りはここで落ちる
  3. 速度が数値微分と一致                            誤差 < 1e-6
  4. 加速度が数値微分と一致（複数姿勢）              誤差 < 1e-4
  5. 伝達比の解析式が IK の数値微分と一致            誤差 < 1e-6
  6. r4 = 26 mm でも 1〜5 が通る                     r1 = r4 の簡約が無いことの検査
  7. 退化ケース（平行四辺形・r1 = 0）
  8. 到達不能で例外が上がる（黙って NaN を返さない）

    python3 -m pytest scripts/test_knee_fourbar.py -v
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from knee_fourbar import (              # noqa: E402,I100
    DeadPoint, Unreachable, branches_from_pose, knee_fourbar,
)

D = math.degrees
R = math.radians

#: 依頼が指定するスイープ。0.5° 刻みで全点見る。
SWEEP_DEG = [160.0 + 0.5 * i for i in range(221)]
#: 加速度は姿勢で式の壊れ方が変わる。230° 付近は誤った式でも偶然近い値が出るので
#: 1 点だけの一致で判断しない。
POSES_DEG = [170.0, 200.0, 230.0, 260.0, 185.0, 245.0]

#: 本機の寸法（r1 = r4 = 20）と、その簡約が効かない寸法（r4 = 26）。
#: 後者を通すのがテスト 6 で、r1 = r4 に暗黙依存した簡約が紛れ込んでいないことの検査。
LINKS = [
    pytest.param(knee_fourbar(), id="r4=20(実機)"),
    pytest.param(knee_fourbar(r4=26.0), id="r4=26"),
]

H = 1e-4        # 数値微分の刻み [rad 相当は deg で取ってから rad に直す]


def dtheta(a, b):
    """角の差を (−180, 180] に畳んで返す [deg]。"""
    return D(math.atan2(math.sin(R(a - b)), math.cos(R(a - b))))


# ---------------------------------------------------------------- 1. ループ
@pytest.mark.parametrize("link", LINKS)
def test_loop_closes(link):
    """FK の出力 (θ2, θ3, θ4) で A + r3·e(θ3) − B の残差 < 1e-9 mm。"""
    worst = 0.0
    for d2 in SWEEP_DEG:
        pose = link.fk(R(d2))
        worst = max(worst, pose.loop_residual(link.r3))
        # 拘束 |B − A| = r3 そのものも見る
        d = pose.coupler
        worst = max(worst, abs(math.hypot(*d) - link.r3))
    assert worst < 1e-9, f"ループが閉じない: 残差 {worst:.3e} mm"


# ------------------------------------------------------ 2. ラウンドトリップ
@pytest.mark.parametrize("link", LINKS)
def test_roundtrip(link):
    """IK(FK(θ2)) == θ2 が誤差 < 1e-9 度。枝を間違えているとここで落ちる。"""
    worst = 0.0
    for d2 in SWEEP_DEG:
        pose = link.fk(R(d2))
        worst = max(worst, abs(dtheta(D(link.ik(pose.theta4).theta2), d2)))
    assert worst < 1e-9, f"往復しない（枝の取り違えを疑う）: {worst:.3e} deg"


@pytest.mark.parametrize("link", LINKS)
def test_roundtrip_theta4_side(link):
    """逆向き（FK(IK(θ4)) == θ4）も同じ精度で閉じること。"""
    worst = 0.0
    for d2 in SWEEP_DEG:
        d4 = link.fk(R(d2)).theta4
        worst = max(worst, abs(dtheta(D(link.fk(link.ik(d4).theta2).theta4), D(d4))))
    assert worst < 1e-9, f"θ4 側の往復が閉じない: {worst:.3e} deg"


def test_wrong_branch_is_detected():
    """枝を逆に取ると往復が大きくずれること（テスト 2 が本当に効いている確認）。

    既存 MATLAB 実装は β の符号が逆（-acos が +acos であるべきところ）で、
    最大 102 度ずれる。テスト 2 はその取り違えを一発で拾う。
    """
    bad = knee_fourbar(beta=+1)          # ε はそのまま = 組み方と矛盾する枝
    worst = max(abs(dtheta(D(bad.ik(bad.fk(R(d2)).theta4).theta2), d2))
                for d2 in SWEEP_DEG)
    assert worst > 10.0, "枝を逆にしても往復してしまう＝テスト 2 が効いていない"


def test_branches_from_measured_pose():
    """実測 1 姿勢から枝が復元できること（実機での確定手順）。"""
    link = knee_fourbar()
    for d2 in (180.0, 220.0, 260.0):
        pose = link.fk(R(d2))
        assert branches_from_pose(pose.theta2, pose.theta4, link) == (link.beta, link.eps)


# -------------------------------------------------------------- 3. 速度
@pytest.mark.parametrize("link", LINKS)
def test_velocity_matches_numeric(link):
    """ω2 = 1 のとき (ω3, ω4) が θ2 の中心差分と一致すること（誤差 < 1e-6）。"""
    h = H
    worst3 = worst4 = 0.0
    for d2 in POSES_DEG:
        t2 = R(d2)
        pose = link.fk(t2)
        w3, w4 = link.velocity(pose, 1.0)
        p, m = link.fk(t2 + h), link.fk(t2 - h)
        n3 = (math.atan2(math.sin(p.theta3 - m.theta3),
                         math.cos(p.theta3 - m.theta3))) / (2.0 * h)
        n4 = (math.atan2(math.sin(p.theta4 - m.theta4),
                         math.cos(p.theta4 - m.theta4))) / (2.0 * h)
        worst3, worst4 = max(worst3, abs(w3 - n3)), max(worst4, abs(w4 - n4))
    assert worst3 < 1e-6, f"ω3 が数値微分と合わない: {worst3:.3e}"
    assert worst4 < 1e-6, f"ω4 が数値微分と合わない: {worst4:.3e}"


# -------------------------------------------------------------- 4. 加速度
@pytest.mark.parametrize("link", LINKS)
@pytest.mark.parametrize("d2", POSES_DEG)
def test_acceleration_matches_numeric(link, d2):
    """ω2 = α2 = 1 のとき (α3, α4) が 2 階の数値微分と一致すること（誤差 < 1e-4）。

    θ2(t) = θ2 + t + t²/2 （ω2 = α2 = 1）を入れて θ3, θ4 を t で 2 階微分する。
    M の行の符号と右辺の符号を片方だけ −1 倍した実装は、速度は通るのにここで落ちる。
    姿勢を 1 点だけ見て判断しないこと（230° 付近は誤った式でも偶然近い値が出る）。
    """
    h = H
    t2 = R(d2)

    def at(t):
        return link.fk(t2 + t + 0.5 * t * t)

    pose = at(0.0)
    a3, a4 = link.acceleration(pose, 1.0, 1.0)
    p, c, m = at(h), pose, at(-h)
    for got, f in ((a3, lambda q: q.theta3), (a4, lambda q: q.theta4)):
        # 角の跳びを避けるため、中心値からの差を (−π, π] に畳んでから 2 階差分する
        dp = math.atan2(math.sin(f(p) - f(c)), math.cos(f(p) - f(c)))
        dm = math.atan2(math.sin(f(m) - f(c)), math.cos(f(m) - f(c)))
        num = (dp + dm) / (h * h)
        assert abs(got - num) < 1e-4, (
            f"θ2 = {d2}° で加速度が数値微分と合わない: 解析 {got:.9f} / 数値 {num:.9f}")


def test_acceleration_sign_pairing_is_checked():
    """M の行と右辺の符号を片方だけ反転すると加速度が壊れることの確認。

    速度は M と右辺を同時に −1 倍しても解が変わらない（両辺に掛かるので消える）。
    壊れるのは加速度側だけなので、テスト 4 が無いと取り違えが素通りする。
    """
    link = knee_fourbar()
    worst_v = worst_a = 0.0
    for d2 in POSES_DEG:
        pose = link.fk(R(d2))
        w3, w4 = link.velocity(pose, 1.0)
        # 右辺だけ符号を反転した「壊れた」加速度
        s2, c2 = math.sin(pose.theta2), math.cos(pose.theta2)
        s3, c3 = math.sin(pose.theta3), math.cos(pose.theta3)
        s4, c4 = math.sin(pose.theta4), math.cos(pose.theta4)
        b1 = -(link.r2 * s2 + link.r2 * c2 + link.r3 * w3 * w3 * c3
               - link.r4 * w4 * w4 * c4)
        b2 = -(-link.r2 * c2 + link.r2 * s2 + link.r3 * w3 * w3 * s3
               - link.r4 * w4 * w4 * s4)
        bad3, bad4 = link._solve2(pose, b1, b2)
        good3, good4 = link.acceleration(pose, 1.0, 1.0)
        worst_a = max(worst_a, abs(bad4 - good4))
        # 速度側は M と右辺を揃えて反転しても不変
        v = link._solve2(pose, -link.r2 * s2, link.r2 * c2)
        worst_v = max(worst_v, abs(-v[1] - w4))
    assert worst_v < 1e-12, "速度は符号の同時反転で不変のはず"
    assert worst_a > 1.0, "右辺だけ反転しても加速度が変わらない＝テスト 4 が効いていない"


# ------------------------------------------------------------ 5. 伝達比
@pytest.mark.parametrize("link", LINKS)
def test_ratio_matches_numeric(link):
    """解析式 dθ4/dθ2 が IK の数値微分と一致すること（誤差 < 1e-6）。

    IK は θ4 -> θ2 なので、その微分 dθ2/dθ4 の逆数と比べる。
    """
    h = H
    worst = 0.0
    for d2 in SWEEP_DEG:
        pose = link.fk(R(d2))
        got = link.ratio(pose)
        p, m = link.ik(pose.theta4 + h).theta2, link.ik(pose.theta4 - h).theta2
        num = (2.0 * h) / math.atan2(math.sin(p - m), math.cos(p - m))
        worst = max(worst, abs(got - num))
    assert worst < 1e-6, f"伝達比が数値微分と合わない: {worst:.3e}"


@pytest.mark.parametrize("link", LINKS)
def test_ratio_matches_fk_numeric(link):
    """FK 側の数値微分とも一致すること（向きの取り違え避け）。"""
    h = H
    worst = 0.0
    for d2 in POSES_DEG:
        t2 = R(d2)
        pose = link.fk(t2)
        p, m = link.fk(t2 + h).theta4, link.fk(t2 - h).theta4
        num = math.atan2(math.sin(p - m), math.cos(p - m)) / (2.0 * h)
        worst = max(worst, abs(link.ratio(pose) - num))
    assert worst < 1e-6, f"伝達比が FK の数値微分と合わない: {worst:.3e}"


# ------------------------------------------------------- 6. 寸法の一般性
def test_r1_ne_r4_changes_result():
    """r1 = r4 に乗った簡約を使っていたら答えが変わることの確認。

    本機は偶然 r1 = r4 = 20 で、そのとき O4–O2–B が二等辺三角形になるので
        |w| = √(2·r1²·(1 − cos θ4))、atan2(w_y, w_x) = (θ4 + π)/2
    が成り立つ。r4 = 26 にすると前者は 5.6 mm、後者は 3.3° ずれる。
    エラーも警告も出ずに間違った角度を返すので、簡約が入っていないことを直接見る。
    """
    link = knee_fourbar(r4=26.0)
    worst_len = worst_ang = 0.0
    for d2 in SWEEP_DEG:
        t4 = link.fk(R(d2)).theta4
        B = link.rocker_pin(t4)
        w = (B[0] - link.r1, B[1])
        wn = math.hypot(*w)
        naive_len = math.sqrt(max(0.0, 2.0 * link.r1 ** 2 * (1.0 - math.cos(t4))))
        worst_len = max(worst_len, abs(wn - naive_len))
        worst_ang = max(worst_ang,
                        abs(dtheta(D(math.atan2(w[1], w[0])), D((t4 + math.pi) / 2.0))))
    assert worst_len > 5.0, f"r1 = r4 の簡約とのずれが小さすぎる: {worst_len:.3f} mm"
    assert worst_ang > 3.0, f"角のずれが小さすぎる: {worst_ang:.3f} deg"


@pytest.mark.parametrize("r4", [14.0, 20.0, 26.0, 33.0])
def test_generic_dimensions_roundtrip(r4):
    """r4 を振っても往復とループが保たれること。"""
    link = knee_fourbar(r4=r4)
    worst_rt = worst_loop = 0.0
    for d2 in SWEEP_DEG:
        if not link.is_assemblable_theta2(R(d2)):
            continue
        pose = link.fk(R(d2))
        worst_loop = max(worst_loop, pose.loop_residual(link.r3))
        worst_rt = max(worst_rt, abs(dtheta(D(link.ik(pose.theta4).theta2), d2)))
    assert worst_loop < 1e-9 and worst_rt < 1e-9


# ------------------------------------------------------------ 7. 退化ケース
def test_parallelogram_is_identity():
    """平行四辺形（r2 = r4 かつ r3 = r1）で θ4 = θ2 の恒等写像になること。

    平行四辺形は θ2 = 0°, 180° で 4 本が一直線に並ぶ「変化点」を持ち、そこで 2 交点が
    重なって枝が入れ替わる。恒等写像を与える枝は (0°, 180°) では (β, ε) = (+1, −1)、
    (180°, 360°) では (−1, +1) になる。枝を姿勢ごとに選び直さないという約束は変わらず、
    変化点をまたがない範囲を取れば定数のままである。本機の寸法は非 Grashof で作動域に
    変化点が無いので（test_no_change_point_in_working_range）この入れ替えは起きない。
    もう一方の枝は交差平行四辺形で、そちらは非線形な写像になる。
    """
    for beta, eps, sweep in ((+1, -1, [2.0 + 2.0 * i for i in range(89)]),
                             (-1, +1, [182.0 + 2.0 * i for i in range(89)])):
        link = knee_fourbar(r1=20.0, r2=45.0, r3=20.0, r4=45.0, beta=beta, eps=eps)
        worst_fk = worst_ik = 0.0
        for d2 in sweep:
            t2 = R(d2)
            worst_fk = max(worst_fk, abs(dtheta(D(link.fk(t2).theta4), d2)))
            worst_ik = max(worst_ik, abs(dtheta(D(link.ik(t2).theta2), d2)))
        assert worst_fk < 1e-9, f"平行四辺形で FK が恒等写像でない: {worst_fk:.3e} deg"
        assert worst_ik < 1e-9, f"平行四辺形で IK が恒等写像でない: {worst_ik:.3e} deg"
        # 恒等写像なら伝達比は 1
        assert abs(link.ratio(link.fk(R(sweep[20]))) - 1.0) < 1e-9
        # 枝は組み方で決まる: 実測 1 姿勢から同じ (β, ε) が復元される
        pose = link.fk(R(sweep[20]))
        assert branches_from_pose(pose.theta2, pose.theta4, link) == (beta, eps)


def test_no_change_point_in_working_range():
    """本機の寸法では作動域に変化点（arccos の引数が ±1）が無く、枝が定数でよいこと。"""
    link = knee_fourbar()
    worst = 0.0
    for d2 in SWEEP_DEG:
        A = link.crank_pin(R(d2))
        d = math.hypot(*A)
        worst = max(worst, abs((link.r4 ** 2 + d * d - link.r3 ** 2) / (2.0 * link.r4 * d)))
    assert worst < 0.999, f"作動域が死点に近づいている: max|G/(r4·d)| = {worst:.6f}"


def test_crossed_parallelogram_is_not_identity():
    """もう一方の枝（交差平行四辺形）は恒等写像にならないこと。"""
    link = knee_fourbar(r1=20.0, r2=45.0, r3=20.0, r4=45.0, beta=-1, eps=+1)
    worst = max(abs(dtheta(D(link.fk(R(d2)).theta4), d2))
                for d2 in [40.0 + 4.0 * i for i in range(70)])
    assert worst > 10.0, "交差平行四辺形まで恒等写像になっている"


@pytest.mark.parametrize("beta,eps", [(-1, +1), (+1, -1)])
def test_coaxial_gives_constant_offset(beta, eps):
    """r1 = 0（モータ軸が膝軸と同軸）で θ2 − θ4 が一定になること。

    クランクとロッカーが同じ点に留まり、カプラが両者の相対角を固定しているだけなので
    直結と等価になる。cos(θ2 − θ4) = (r2² + r4² − r3²)/(2·r2·r4) から予測値も出せる。
    """
    link = knee_fourbar(r1=0.0, r2=45.0, r3=35.0, r4=20.0, beta=beta, eps=eps)
    want = math.acos((link.r2 ** 2 + link.r4 ** 2 - link.r3 ** 2)
                     / (2.0 * link.r2 * link.r4))
    offs = [dtheta(D(link.fk(R(d2)).theta4), d2) for d2 in [0.0 + 3.0 * i for i in range(120)]]
    spread = max(offs) - min(offs)
    assert spread < 1e-9, f"θ2 − θ4 が一定でない: ばらつき {spread:.3e} deg"
    assert abs(abs(offs[0]) - D(want)) < 1e-9, "オフセットが余弦定理と合わない"
    # 逆変換も同じ一定オフセットを戻すこと
    worst = max(abs(dtheta(D(link.ik(link.fk(R(d2)).theta4).theta2), d2))
                for d2 in [0.0 + 3.0 * i for i in range(120)])
    assert worst < 1e-9


# --------------------------------------------------------- 8. 到達不能の扱い
def test_unreachable_raises():
    """θ2 = 0° は三角形が閉じない。例外が上がり、黙って NaN を返さないこと。"""
    link = knee_fourbar()
    assert not link.is_assemblable_theta2(0.0)
    with pytest.raises(Unreachable):
        link.fk(0.0)


@pytest.mark.parametrize("d2", [0.0, 30.0, 60.0, 70.0, 300.0, 350.0])
def test_unreachable_theta2_never_returns_nan(d2):
    """組めない θ2 で NaN が返らないこと（例外か、有限の値か）。"""
    link = knee_fourbar()
    try:
        pose = link.fk(R(d2))
    except Unreachable:
        return
    for v in (pose.theta3, pose.theta4):
        assert math.isfinite(v), f"θ2 = {d2}° で NaN が返った"
    assert link.is_assemblable_theta2(R(d2))


def test_unreachable_theta4_raises():
    """逆変換側も同じ。|S| > |w| で例外が上がること。"""
    link = knee_fourbar()
    # |w| ≤ r2 − r3 = 10 になる θ4 は無い（|w| ≥ r2 − ... ）ので |w| > r2 + r3 側を作る
    far = knee_fourbar(r1=200.0)
    assert not far.is_assemblable_theta4(0.0)
    with pytest.raises(Unreachable):
        far.ik(0.0)
    # 通常寸法で組める範囲の外
    assert not link.is_assemblable_theta2(R(0.0))


def test_dead_point_raises_not_nan():
    """死点では伝達比が発散するので DeadPoint を上げること。"""
    link = knee_fourbar()
    # d = r3 + r4 になる θ2 が死点（カプラとロッカーが一直線）
    c = ((link.r3 + link.r4) ** 2 - link.r1 ** 2 - link.r2 ** 2) / (2 * link.r1 * link.r2)
    pose = link.fk(math.acos(max(-1.0, min(1.0, c))))
    with pytest.raises(DeadPoint):
        link.ratio(pose)
    with pytest.raises(DeadPoint):
        link.velocity(pose, 1.0)


def test_bad_parameters_rejected():
    """不正な寸法・枝は構築時に弾くこと。"""
    for bad in (dict(r2=0.0), dict(r3=-1.0), dict(r1=-1.0), dict(beta=0),
                dict(eps=2), dict(sigma_joint=0), dict(gear=0.0)):
        with pytest.raises(ValueError):
            knee_fourbar(**bad)


# ------------------------------------------------- 参考: 導出との答え合わせ
@pytest.mark.parametrize("link", LINKS)
def test_freudenstein(link):
    """Freudenstein の式 (KN-9) の残差が消えること。導出の答え合わせ。"""
    worst = max(abs(link.freudenstein_residual(link.fk(R(d2)))) for d2 in SWEEP_DEG)
    assert worst < 1e-12, f"Freudenstein の残差 {worst:.3e}"


def test_reference_values():
    """依頼 §5 の参照値を再現すること。"""
    link = knee_fourbar()
    for d2, want3, want4, want_g in ((180.0, 34.05, 78.46, 44.42),
                                     (200.0, 69.34, 119.79, 50.45),
                                     (220.0, 97.69, 163.26, 65.58),
                                     (240.0, 117.05, 202.95, 85.90),
                                     (260.0, 129.45, 239.82, 110.38)):
        pose = link.fk(R(d2))
        assert D(pose.theta3) == pytest.approx(want3, abs=0.01)
        assert D(pose.theta4) == pytest.approx(want4, abs=0.01)
        assert D(link.transmission_angle(pose)) == pytest.approx(want_g, abs=0.01)

    t4 = [D(link.fk(R(d2)).theta4) for d2 in SWEEP_DEG]
    assert min(t4) == pytest.approx(50.53, abs=0.01)
    assert max(t4) == pytest.approx(258.28, abs=0.01)

    # 膝角 0〜120° に必要な θ2 と、その区間の伝達比・伝達角
    m_lo, m_hi = link.ik_deg(90.0), link.ik_deg(210.0)
    assert m_lo == pytest.approx(186.06, abs=0.01)
    assert m_hi == pytest.approx(243.75, abs=0.01)
    assert m_hi - m_lo == pytest.approx(57.69, abs=0.01)
    sub = [link.fk(R(m_lo + (m_hi - m_lo) * i / 500.0)) for i in range(501)]
    rr = [link.ratio(p) for p in sub]
    gg = [D(link.transmission_angle(p)) for p in sub]
    assert min(rr) == pytest.approx(1.867, abs=0.001)
    assert max(rr) == pytest.approx(2.217, abs=0.001)
    assert min(gg) == pytest.approx(45.0, abs=0.05)
    assert max(gg) == pytest.approx(90.2, abs=0.05)

    # 非 Grashof（三揺動）
    sl, pq, is_grashof = link.grashof()
    assert (sl, pq, is_grashof) == (65.0, 55.0, False)


def test_assemblable_interval_is_exact():
    """組める θ2 の区間。境界は cos θ2 = 1/3、すなわち acos(1/3) = 70.5288°。

    依頼 §5 の表は 70.8°〜289.2° としているが、閉形式で解くと
        d ≤ r3 + r4 = 55  ⟺  2425 + 1800·cos θ2 ≤ 3025  ⟺  cos θ2 ≤ 1/3
    なので 70.5288°〜289.4712° が正しい。0.27° ぶん表の方が内側に寄っている。
    """
    link = knee_fourbar()
    want = D(math.acos(1.0 / 3.0))
    assert want == pytest.approx(70.5288, abs=1e-3)
    step = 1e-3
    lo = min(d2 for d2 in [70.0 + step * i for i in range(2000)]
             if link.is_assemblable_theta2(R(d2)))
    assert lo == pytest.approx(want, abs=2e-3)
    assert link.is_assemblable_theta2(R(want + 0.01))
    assert not link.is_assemblable_theta2(R(want - 0.01))


def test_joint_and_motor_layer():
    """関節角・サーボ指令の層が往復すること（暫定の原点・向きのまま）。"""
    link = knee_fourbar()
    worst = 0.0
    for j in [0.0 + 2.0 * i for i in range(61)]:      # 膝角 0〜120°
        motor = link.joint_to_motor_deg(j)
        worst = max(worst, abs(dtheta(link.motor_to_joint_deg(motor), j)))
    assert worst < 1e-9, f"関節角 -> サーボ -> 関節角 が戻らない: {worst:.3e} deg"

    # σ・原点・ギア比を暫定でない値に振っても往復すること
    alt = knee_fourbar(sigma_joint=-1, theta4_zero=R(95.0),
                       phi0=R(30.0), sigma_motor=-1, gear=2.5)
    worst = max(abs(dtheta(alt.motor_to_joint_deg(alt.joint_to_motor_deg(-j)), -j))
                for j in [0.0 + 2.0 * i for i in range(61)])
    assert worst < 1e-9


# ------------------------------------------- 9. 確定値（T ポーズ）と σ の一意性
def test_theta4_zero_is_tpose():
    """θ4_zero = 89.3° が「脚が伸び切った姿勢」であること。

    knee_config の確定値そのものと、その姿勢で 4 節リンクが組めることを見る。
    脚全体が本当に伸び切っているかは test_leg_bridge_tpose が脚 IK 側で確かめる。
    """
    link = knee_fourbar()
    assert D(link.theta4_zero) == pytest.approx(89.3, abs=1e-12)
    assert link.sigma_joint == +1
    pose = link.ik(link.rocker_from_joint(0.0))
    assert D(pose.theta2) == pytest.approx(185.7067, abs=1e-3)
    assert link.ratio(pose) == pytest.approx(1.990, abs=1e-3)


def test_sigma_is_unique():
    """σ_knee は θ4_zero と枝から一意に決まり、選択の余地がないこと。

    σ = -1 は屈曲 60.3°（θ4 = 28.96°）で死点に当たる。そこでは伝達比が 0.03 まで
    落ちてサーボが膝を動かせないので、設計可動域 0-150° に届くのは +1 だけ。
    """
    reach = {}
    for sig in (+1, -1):
        link = knee_fourbar(sigma_joint=sig)
        got, rmin = 0.0, float("inf")
        for i in range(1501):
            try:
                p = link.ik(link.rocker_from_joint(R(i / 10.0)))
                rmin = min(rmin, abs(link.ratio(p)))
            except (Unreachable, DeadPoint):
                break
            got = i / 10.0
        reach[sig] = (got, rmin)
    assert reach[+1][0] >= 150.0, "σ = +1 が設計可動域に届かない"
    assert reach[-1][0] < 65.0, "σ = -1 でも届いてしまう＝一意性が崩れている"
    assert reach[-1][1] < 0.1, "σ = -1 が死点に当たっていない"
    assert reach[+1][1] > 1.8, "σ = +1 の作動域で伝達比が落ちている"


def test_design_range_with_confirmed_zero():
    """確定した θ4_zero での設計可動域（膝 0-120°）の値。"""
    link = knee_fourbar()
    sub = [link.ik(link.rocker_from_joint(R(120.0 * i / 500.0))) for i in range(501)]
    lo, hi = D(sub[0].theta2), D(sub[-1].theta2)
    assert lo == pytest.approx(185.71, abs=0.01)
    assert hi == pytest.approx(243.37, abs=0.01)
    assert hi - lo == pytest.approx(57.67, abs=0.01)
    rr = [link.ratio(p) for p in sub]
    gg = [D(link.transmission_angle(p)) for p in sub]
    assert min(rr) == pytest.approx(1.870, abs=0.001)
    assert max(rr) == pytest.approx(2.217, abs=0.001)
    assert min(gg) > 40.0 and max(gg) < 140.0, "伝達角が設計の目安を外れている"


# ------------------------------------------------------ 10. 脚 IK との繋ぎこみ
def test_leg_bridge_tpose():
    """曲げ量 0 で脚が本当に伸び切ること（T ポーズ）を脚 IK 側で確かめる。

    伸び切りなら股中心 o3 から足首ロール軸 o5 までが ℓ3' + ℓ4 ちょうどになる。
    """
    import numpy as np

    import leg_ik
    from leg_servo import leg_angle_from_knee_bend, leg_servo

    ls = leg_servo("right")
    theta = np.zeros(6)
    theta[3] = leg_angle_from_knee_bend(0.0, ls.leg)
    o = leg_ik.joint_origins(theta, ls.leg)
    assert float(np.linalg.norm(o[2] - o[0])) == pytest.approx(
        ls.leg.l3e + ls.leg.l4, abs=1e-9), "曲げ量 0 で脚が伸び切っていない"
    assert D(ls.knee.rocker_from_joint(0.0)) == pytest.approx(89.3, abs=1e-12)


def test_leg_bridge_roundtrip():
    """脚 IK の公開角 θ4 -> サーボ指令 -> 公開角 が往復すること。"""
    from leg_servo import leg_angle_from_knee_bend, leg_servo

    ls = leg_servo("right")
    worst = 0.0
    servos = []
    for i in range(301):                       # 屈曲 0-150° を 0.5° 刻み
        th4 = leg_angle_from_knee_bend(R(i * 0.5), ls.leg)
        s = ls.knee_servo_from_angle(th4)
        servos.append(s)
        worst = max(worst, abs(dtheta(D(ls.knee_angle_from_servo(s)), D(th4))))
    assert worst < 1e-9, f"関節角 <-> サーボ指令が往復しない: {worst:.3e} deg"
    assert D(max(servos) - min(servos)) == pytest.approx(74.01, abs=0.01)


def test_leg_bridge_with_full_ik():
    """脚 IK 全体と繋いだ往復。fk → ik → サーボ → 関節角。"""
    import numpy as np

    import leg_ik
    from leg_servo import leg_angle_from_knee_bend, leg_servo

    ls = leg_servo("right")
    rng = np.random.default_rng(20260828)
    lo = np.array([leg_ik.JOINT_LIMITS[k][0] for k in leg_ik.JOINT_NAMES])
    hi = np.array([leg_ik.JOINT_LIMITS[k][1] for k in leg_ik.JOINT_NAMES])
    worst, n_ok = 0.0, 0
    for _ in range(500):
        th = rng.uniform(lo, hi)
        th[3] = leg_angle_from_knee_bend(abs(th[3]), ls.leg)
        th[:3] *= ls.leg.sign[:3]
        th[4:] *= ls.leg.sign[4:]
        p, Rm = leg_ik.fk(th, ls.leg)
        try:
            back = leg_ik.ik(p, Rm, ls.leg, clamp=False)
        except leg_ik.Unreachable:
            continue
        again = ls.joints_from_servo(ls.servo_from_joints(back))
        worst = max(worst, abs(dtheta(D(again[3]), D(th[3]))))
        n_ok += 1
    assert n_ok > 400, f"可動域内なのに解けない姿勢が多すぎる ({n_ok}/500)"
    assert worst < 1e-9, f"脚 IK と繋ぐと往復しない: {worst:.3e} deg"


def test_ankle_is_not_silently_passed_through():
    """足首は Python 版が無い。素通しを頼まれたら明示的に断ること。"""
    import numpy as np

    from leg_servo import AnkleNotImplemented, leg_servo

    ls = leg_servo("right")
    tpose = np.zeros(6)                       # 伸び切り姿勢の関節角
    servo = ls.servo_from_joints(tpose)       # ankle 抜きなら素通しで通る
    assert D(servo[3]) == pytest.approx(185.7067, abs=1e-3)
    with pytest.raises(AnkleNotImplemented):
        ls.servo_from_joints(tpose, ankle=True)
    with pytest.raises(AnkleNotImplemented):
        ls.joints_from_servo(servo, ankle=True)
