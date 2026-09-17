# -*- coding: utf-8 -*-
"""静歩行の計画エンジン (static_walk)。

**仕様の原本。** C++ 版 (roboone_walk_core/static_walk_engine.hpp) と JS 版
(roboone_viz/staticwalk.js) はこれの移植で、ロジックを変えるときは 3 つを揃える。
roboone_viz の準静的歩行の試作 (2026-09-17) を昇格させたもの。

walk_core (DCM・純フィードフォワード) は ZMP を 1 歩のあいだ支持足に固定するので、
周期を延ばしても支持足の交代の瞬間は重心が両足の中間を約 ωW/2 で横切る。
ここでは重心を直接計画して、**各瞬間で静的に立っている**歩き方を作る:

    IDLE   両足の中点に重心を置いて立つ
    SHIFT  両足支持のまま、重心を次の支持足の上へ移す (5 次多項式)
    SWING  重心を支持足の上で止めたまま、反対の足を振り出す
    (SHIFT と SWING を繰り返す)
    STOP   足が揃ったあと、重心を両足の中点へ戻して IDLE へ

止まり方: 歩の境界で指令が v_stop_eps 未満なら、足が揃っていればそのまま STOP、
揃っていなければ支持足の真横へ揃える 1 歩 (mode='stop') を踏んでから STOP。
STOP の途中で指令が戻っても、IDLE まで戻り切ってから歩き直す。

重心の加速度が小さければ ZMP は重心の真下からほとんどずれない (LIPM: p = c - c''/ω²)。
重心移動の時間は、そのずれが zmp_tol 以下になるように距離から決める:

    5 次多項式の加速度の最大 = (10/√3) D / τ²  <=  ω² zmp_tol

歩幅は v·stride_time で、**振り出しの開始時に固定**する (遊脚中に指令が変わっても
着地点は動かさない)。純 FF の発散が無いので、walk_core の着地点クランプも DCM 補正も
持たない。指令の整形 (飽和・楕円・a_max) は walk_core と同じ。

出力は walk_core の WalkOutputs をそのまま使う。motion ノードの下流 (IK・安定化・
/motion/stab) は同じ形で読める。

    state    IDLE / SHIFT / SWING / STOP / ESTOP
    phase    SHIFT・STOP では重心移動の経過 [0,1]、SWING では振り出しの経過 [0,1]
    support  SWING 中だけ支持足 (±1)。それ以外は 0 (両足)
    xi, zmp  重心の計画から出す (ξ = c + c'/ω、p = c - c''/ω²)
    p_nom, p_land   SWING 中の着地点 (同じ値)。locked は SWING 中 True
"""

import math
from typing import List, Optional

from .params import StaticGaitParams
from ..walk_core.engine import (ESTOP, IDLE, LEFT, RIGHT, StepRecord, STOP,
                                WalkOutputs)

SHIFT = 'SHIFT'
SWING = 'SWING'

K_QUINTIC = 10.0 / math.sqrt(3.0)   # 5 次多項式の加速度ピーク係数
SWING_RISE = 0.45                   # 振り出しのうち足を上げる区間 (walk_core の遊脚と同じ)

__all__ = ['StaticWalkEngine', 'check_static_gait', 'support_margin',
           'SHIFT', 'SWING', 'IDLE', 'STOP', 'ESTOP', 'LEFT', 'RIGHT']


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _s(u):
    """5 次多項式 s(u) = 10u³ - 15u⁴ + 6u⁵ と、その 1 階・2 階微分。"""
    u = _clamp(u, 0.0, 1.0)
    s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u))
    ds = 30.0 * u * u * (1.0 + u * (-2.0 + u))
    dds = 60.0 * u * (1.0 + u * (-3.0 + 2.0 * u))
    return s, ds, dds


def shift_time(p: StaticGaitParams, dist: float) -> float:
    """重心を dist [m] 動かすのにかける時間。ZMP のずれを zmp_tol に収める。"""
    a_lim = p.omega ** 2 * p.zmp_tol
    return max(p.t_shift_min, math.sqrt(K_QUINTIC * dist / a_lim))


def _swing_ref(p: StaticGaitParams, tau: float):
    """振り出しの経過 tau での足の高さの基準 (飽和前) と、降下区間かどうか。"""
    if tau < SWING_RISE:
        return p.swing_height * _s(tau / SWING_RISE)[0], False
    su = _s((tau - SWING_RISE) / (1.0 - SWING_RISE))[0]
    return p.swing_height * (1.0 - su) - p.td_overdrive * su, True


def swing_height_at(p: StaticGaitParams, tau: float, z_prev: float, dt: float) -> float:
    """振り出しの経過 tau での足の高さ。降下は td_speed_max で飽和する (前周期 z_prev から)。"""
    z_ref, falling = _swing_ref(p, tau)
    return max(z_ref, z_prev - p.td_speed_max * dt) if falling else z_ref


def check_static_gait(p: StaticGaitParams) -> dict:
    """設定が静歩行として成り立つかを調べる。errors が空なら使える。

    遊脚の着地は式で近似せず、エンジンと同じ漸化式 (swing_height_at) を loop_hz で回す。
    """
    dt = 1.0 / (p.loop_hz if p.loop_hz > 0.0 else 200.0)
    r = {'lands': False, 'touch_phase': 1.0, 'saturated': False, 'z_end': 0.0,
         'v_need': (p.swing_height + p.td_overdrive) / ((1.0 - SWING_RISE) * p.t_swing)}
    z = t = 0.0
    while t < p.t_swing - 1e-12:
        t += dt
        tau = min(t / p.t_swing, 1.0)
        z_ref, falling = _swing_ref(p, tau)
        if falling and z - p.td_speed_max * dt > z_ref + 1e-12:
            r['saturated'] = True
        z = swing_height_at(p, tau, z, dt)
        if not r['lands'] and z <= 0.0:
            r['lands'] = True
            r['touch_phase'] = tau
    r['z_end'] = z

    w = p.foot_spacing
    stride = (p.v_max[0] * p.stride_time, p.v_max[1] * p.stride_time)
    # 定常歩行の重心移動は「支持足の上 -> 次の支持足の上」。前後に歩幅、横に W ± 2·offset
    d_fwd = math.hypot(stride[0], w + 2.0 * p.com_offset_y)
    r['t_shift_start'] = shift_time(p, abs(w / 2.0 + p.com_offset_y))
    r['t_shift_step'] = shift_time(p, w + 2.0 * p.com_offset_y)
    r['t_cycle_fwd'] = shift_time(p, d_fwd) + p.t_swing
    r['v_fwd_real'] = stride[0] / r['t_cycle_fwd']
    r['stride_max'] = stride
    r['margin_single'] = (p.sole_length / 2.0, p.sole_width / 2.0 - abs(p.com_offset_y))

    err = []
    if not r['lands']:
        err.append(
            f"遊脚が床に届かないまま振り出しが終わる (+{r['z_end'] * 1000:.1f}mm 浮いている)。"
            f"td_speed_max を {r['v_need']:.3f} 以上にするか、swing_height を下げるか、"
            't_swing を伸ばすこと')
    if r['margin_single'][1] <= p.zmp_tol:
        err.append(
            f'com_offset_y {p.com_offset_y * 1000:.1f}mm では片足支持の重心が足裏の縁'
            f' (半幅 {p.sole_width * 500:.1f}mm) から zmp_tol 以内に出る')
    if p.zmp_tol <= 0.0 or p.t_swing <= 0.0 or p.stride_time <= 0.0:
        err.append('zmp_tol / t_swing / stride_time は正の値にすること')
    if p.foot_spacing <= p.sole_width:
        err.append(f'足間隔 {w * 1000:.0f}mm が足裏の幅 {p.sole_width * 1000:.0f}mm 以下 (足が重なる)')
    r['errors'] = err
    return r


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _convex_hull(pts):
    pts = sorted(set(pts))
    if len(pts) <= 2:
        return pts
    lower, upper = [], []
    for q in pts:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], q) <= 0.0:
            lower.pop()
        lower.append(q)
    for q in reversed(pts):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], q) <= 0.0:
            upper.pop()
        upper.append(q)
    return lower[:-1] + upper[:-1]      # 反時計回り


def _seg_dist(pt, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    L2 = ax * ax + ay * ay
    t = 0.0 if L2 <= 0.0 else _clamp(((pt[0] - a[0]) * ax + (pt[1] - a[1]) * ay) / L2, 0.0, 1.0)
    return math.hypot(pt[0] - (a[0] + t * ax), pt[1] - (a[1] + t * ay))


def support_margin(out: WalkOutputs, p: StaticGaitParams, point=None) -> float:
    """ZMP (point を渡せばその点) が支持多角形の縁からどれだけ内側にあるか [m]。負なら外。

    支持多角形は床に着いている足 (z <= 0) の足裏長方形の凸包。足のヨーは 0。
    """
    pt = out.zmp if point is None else point
    hx, hy = p.sole_length / 2.0, p.sole_width / 2.0
    corners = []
    for f in (out.left_foot, out.right_foot):
        if f[2] > 1e-9:
            continue
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                corners.append((f[0] + sx * hx, f[1] + sy * hy))
    hull = _convex_hull(corners)
    if len(hull) < 3:
        return -math.inf
    inside = True
    d_min = math.inf
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        if _cross(a, b, pt) < 0.0:
            inside = False
        d_min = min(d_min, _seg_dist(pt, a, b))
    return d_min if inside else -d_min


class StaticWalkEngine:
    """update(vx, vy, dt) を 200 Hz で回す静歩行の計画器。"""

    def __init__(self, params: Optional[StaticGaitParams] = None):
        self.p = params or StaticGaitParams()
        self.reset()

    # ------------------------------------------------------------------ 初期化
    def reset(self):
        w2 = self.p.foot_spacing / 2.0
        self.t = 0.0
        self.state = IDLE
        self.step_idx = 0
        self.v = [0.0, 0.0]
        self.foot = {LEFT: [0.0, +w2], RIGHT: [0.0, -w2]}
        self.sup = LEFT
        self.com = [0.0, 0.0]
        self.comv = [0.0, 0.0]
        self.coma = [0.0, 0.0]
        self.phase = 0.0
        self.t_local = 0.0
        self.dur = 0.0
        self.c0 = [0.0, 0.0]
        self.c1 = [0.0, 0.0]
        self.swing_r0 = [0.0, 0.0]
        self.swing_z = 0.0
        self.p_land = None
        self.mode = 'walk'
        self.steps: List[StepRecord] = []

    # ------------------------------------------------------------------ 補助
    def _midpoint(self):
        a, b = self.foot[LEFT], self.foot[RIGHT]
        return [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]

    def _moving(self, eps):
        """整形後の指令が eps 以上か。viz の足踏みの包みがここを差し替える。"""
        return math.hypot(self.v[0], self.v[1]) >= eps

    def _feet_aligned(self):
        a, b = self.foot[LEFT], self.foot[RIGHT]
        return (abs(a[0] - b[0]) < 1e-9 and
                abs((a[1] - b[1]) - self.p.foot_spacing) < 1e-9)

    def _over_support(self):
        """振り出し中に重心を置く点。支持足の中心から外側へ com_offset_y。"""
        f = self.foot[self.sup]
        return [f[0], f[1] + self.sup * self.p.com_offset_y]

    def _shape_cmd(self, vx, vy, dt):
        """walk_core の _shape_cmd と同じ (飽和・楕円・a_max)。"""
        p = self.p
        vx = _clamp(vx, -p.v_max[0], p.v_max[0])
        vy = _clamp(vy, -p.v_max[1], p.v_max[1])
        s = math.hypot(vx / p.v_max[0], vy / p.v_max[1])
        if s > 1.0:
            vx /= s
            vy /= s
        for k, (v_in, a) in enumerate(zip((vx, vy), p.a_max)):
            self.v[k] += _clamp(v_in - self.v[k], -a * dt, a * dt)

    # ------------------------------------------------------------------ 1 周期
    def update(self, vx_cmd: float, vy_cmd: float, dt: float,
               estop: bool = False) -> WalkOutputs:
        self.t += dt
        self._shape_cmd(vx_cmd, vy_cmd, dt)
        if estop:
            self.state = ESTOP
        if self.state == ESTOP:
            return self._outputs()     # 脱力。復帰は reset() (実機では home 技) から
        if self.state == IDLE:
            self._tick_idle()
        elif self.state in (SHIFT, STOP):
            self._tick_shift(dt)
        elif self.state == SWING:
            self._tick_swing(dt)
        return self._outputs()

    def _tick_idle(self):
        self.com = self._midpoint()
        self.comv = [0.0, 0.0]
        self.coma = [0.0, 0.0]
        self.phase = 0.0
        if self._moving(self.p.v_start_eps):
            # 横移動は進行方向側の足から踏み出す (walk_core と同じ)
            vy = self.v[1]
            if abs(vy) > 1e-6:
                self.sup = RIGHT if vy > 0 else LEFT
            else:
                self.sup = LEFT
            self._start_shift(self._over_support(), SHIFT)

    # ------------------------------------------------------------ 重心移動
    def _start_shift(self, target, state):
        self.state = state
        self.c0 = list(self.com)
        self.c1 = [target[0], target[1]]
        d = math.hypot(self.c1[0] - self.c0[0], self.c1[1] - self.c0[1])
        self.dur = shift_time(self.p, d)
        self.t_local = 0.0
        self.phase = 0.0

    def _tick_shift(self, dt):
        self.t_local = min(self.t_local + dt, self.dur)
        u = self.t_local / self.dur
        s, ds, dds = _s(u)
        for k in (0, 1):
            dc = self.c1[k] - self.c0[k]
            self.com[k] = self.c0[k] + dc * s
            self.comv[k] = dc * ds / self.dur
            self.coma[k] = dc * dds / (self.dur * self.dur)
        self.phase = u
        if self.t_local < self.dur:
            return
        self.com = list(self.c1)
        self.comv = [0.0, 0.0]
        self.coma = [0.0, 0.0]
        if self.state == STOP:
            self.state = IDLE
            self.phase = 0.0
        elif not self._moving(self.p.v_stop_eps) and self._feet_aligned():
            # 足が揃ったまま指令が消えた: 振り出さずに中点へ戻る
            self._start_shift(self._midpoint(), STOP)
        else:
            self._start_swing()

    # -------------------------------------------------------------- 振り出し
    def _start_swing(self):
        p = self.p
        self.state = SWING
        self.step_idx += 1
        self.t_local = 0.0
        self.dur = p.t_swing
        self.phase = 0.0
        swing = -self.sup
        self.swing_r0 = list(self.foot[swing])
        self.swing_z = 0.0
        ps = self.foot[self.sup]
        if self._moving(p.v_stop_eps):
            self.mode = 'walk'
            self.p_land = [ps[0] + self.v[0] * p.stride_time,
                           ps[1] + swing * p.foot_spacing + self.v[1] * p.stride_time]
        else:
            self.mode = 'stop'     # 支持足の真横に揃える
            self.p_land = [ps[0], ps[1] + swing * p.foot_spacing]
        self.steps.append(StepRecord(
            step_idx=self.step_idx, t_start=self.t, support=self.sup,
            mode=self.mode, v=tuple(self.v), p_support=tuple(ps),
            p_nom=tuple(self.p_land), p_land=tuple(self.p_land)))

    def _tick_swing(self, dt):
        self.t_local = min(self.t_local + dt, self.dur)
        self.phase = self.t_local / self.dur
        tau = self.phase
        s = _s(tau)[0]
        swing = -self.sup
        f = self.foot[swing]
        f[0] = self.swing_r0[0] + s * (self.p_land[0] - self.swing_r0[0])
        f[1] = self.swing_r0[1] + s * (self.p_land[1] - self.swing_r0[1])
        self.swing_z = swing_height_at(self.p, tau, self.swing_z, dt)
        if self.t_local < self.dur:
            return
        # 着地
        f[0], f[1] = self.p_land
        self.swing_z = 0.0
        self.steps[-1].t_end = self.t
        self.p_land = None
        if self.mode == 'stop' or (
                not self._moving(self.p.v_stop_eps) and self._feet_aligned()):
            # 足が揃った: 重心を中点へ戻して止まる
            self._start_shift(self._midpoint(), STOP)
        else:
            self.sup = swing
            self._start_shift(self._over_support(), SHIFT)

    # ------------------------------------------------------------------ 出力
    def _outputs(self) -> WalkOutputs:
        w = self.p.omega
        swinging = self.state == SWING
        swing = -self.sup
        xi = tuple(self.com[k] + self.comv[k] / w for k in (0, 1))
        zmp = tuple(self.com[k] - self.coma[k] / (w * w) for k in (0, 1))
        lf, rf = self.foot[LEFT], self.foot[RIGHT]
        return WalkOutputs(
            t=self.t, state=self.state, step_idx=self.step_idx, phase=self.phase,
            support=self.sup if swinging else 0,
            v=tuple(self.v), xi=xi, com=tuple(self.com), zmp=zmp,
            left_foot=(lf[0], lf[1], self.swing_z if swinging and swing == LEFT else 0.0),
            right_foot=(rf[0], rf[1], self.swing_z if swinging and swing == RIGHT else 0.0),
            pelvis=(self.com[0], self.com[1], self.p.z_c),
            p_nom=tuple(self.p_land) if swinging else None,
            p_land=tuple(self.p_land) if swinging else None,
            locked=swinging,
            stopping=(swinging and self.mode == 'stop') or self.state == STOP)
