# -*- coding: utf-8 -*-
"""準静的歩行の試作 (可視化専用)。

**実機の歩行エンジン (roboone_walk_core) にも、仕様原本の walk_core にも入っていない。**
motion ノードはこれを読まない。シミュレータで見るためだけの計画器。

walk_core (DCM・純フィードフォワード) は ZMP を 1 歩のあいだ支持足に固定するので、
周期を延ばしても支持足の交代の瞬間は重心が両足の中間を約 ωW/2 (0.43 m/s) で横切る。
ここでは重心を直接計画して、各瞬間で静的に立っている歩き方を作る:

    SHIFT  両足支持のまま、重心を次の支持足の真上へ移す (5 次多項式)
    SWING  重心を支持足の真上で止めたまま、反対の足を振り出す
    (繰り返し)
    STOP   足を揃えたあと、重心を両足の中点へ戻す

重心の加速度が小さければ ZMP は重心の真下からほとんどずれない (LIPM: p = c - c''/ω²)。
重心移動の時間は、そのずれが qs.zmp_tol 以下になるように距離から決める:

    5 次多項式の加速度の最大 = (10/√3) D / τ²  <=  ω² zmp_tol

★振り出す足は、骨盤から横へ足間隔 W だけ離れたまま持ち上がる。足首のパラレルリンクは
  外へ開いた足を高く上げられない (z_c=261 で 横 140mm なら 22mm、横 100mm なら 50mm まで。
  2026-09-17 に leg_service で走査)。qs.com_inset で重心を支持足の中心から内側へ寄せるか、
  qs.swing_height で足上げを下げると届く。寄せたぶん静的な余裕 (足裏の半幅 - 寄せ量) は減る。

歩幅は walk_core と同じく L = v·T (T = gait.t_step)。足跡は動的歩行と同じで、
1 歩にかかる時間だけが長い。指令の整形 (飽和・楕円・a_max) も walk_core と同じ。
march=True の間は指令ゼロでも歩き続ける (MarchWalkEngine と同じ約束)。

JS 版は quasistatic.js。ロジックを変えるときは両方を揃える。
"""

from dataclasses import asdict, dataclass
import math
from typing import List, Optional

from roboone_walk_ref.walk_core import GaitParams
from roboone_walk_ref.walk_core.engine import StepRecord, WalkOutputs

LEFT = +1
RIGHT = -1
K_QUINTIC = 10.0 / math.sqrt(3.0)   # 5 次多項式の加速度ピーク係数


@dataclass
class QsParams:
    zmp_tol: float = 0.005     # [m] 重心の加速度で ZMP が重心の真下からずれてよい量
    t_swing: float = 0.8       # [s] 振り出しの時間 (重心は支持足の上で止めておく)
    t_shift_min: float = 0.3   # [s] 重心移動の最短時間
    com_inset: float = 0.0     # [m] 振り出し中の重心を支持足の中心から内側 (もう片足の側) へ寄せる量
    swing_height: Optional[float] = None   # [m] 足上げの高さ。None なら gait.swing_height

    def to_dict(self) -> dict:
        return asdict(self)


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _s(u):
    """5 次多項式 s(u) = 10u³ - 15u⁴ + 6u⁵ と、その 1 階・2 階微分。"""
    u = _clamp(u, 0.0, 1.0)
    s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u))
    ds = 30.0 * u * u * (1.0 + u * (-2.0 + u))
    dds = 60.0 * u * (1.0 + u * (-3.0 + 2.0 * u))
    return s, ds, dds


class QuasiStaticWalker:
    """update(vx, vy, dt, march=...) を回す準静的歩行の計画器 (可視化専用)。"""

    def __init__(self, params: Optional[GaitParams] = None,
                 qs: Optional[QsParams] = None):
        self.p = params or GaitParams()
        self.q = qs or QsParams()
        self.h_sw = (self.p.swing_height if self.q.swing_height is None
                     else self.q.swing_height)
        self.reset()

    # ------------------------------------------------------------------ 初期化
    def reset(self):
        w2 = self.p.foot_spacing / 2.0
        self.t = 0.0
        self.state = 'IDLE'
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
    def _omega2(self):
        return self.p.gravity / self.p.z_c

    def _midpoint(self):
        a, b = self.foot[LEFT], self.foot[RIGHT]
        return [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]

    def _moving(self, eps, march):
        return march or math.hypot(self.v[0], self.v[1]) >= eps

    def _feet_aligned(self):
        a, b = self.foot[LEFT], self.foot[RIGHT]
        return (abs(a[0] - b[0]) < 1e-9 and
                abs((a[1] - b[1]) - self.p.foot_spacing) < 1e-9)

    def _over_support(self):
        """振り出し中に重心を置く点。支持足の中心から内側へ com_inset。"""
        f = self.foot[self.sup]
        return [f[0], f[1] - self.sup * self.q.com_inset]

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
    def update(self, vx_cmd, vy_cmd, dt, estop=False, march=False) -> WalkOutputs:
        self.t += dt
        self._shape_cmd(vx_cmd, vy_cmd, dt)
        if estop:
            self.state = 'ESTOP'
        if self.state == 'ESTOP':
            return self._outputs()
        if self.state == 'IDLE':
            self._tick_idle(march)
        elif self.state in ('SHIFT', 'STOP'):
            self._tick_shift(dt, march)
        elif self.state == 'SWING':
            self._tick_swing(dt, march)
        return self._outputs()

    def _tick_idle(self, march):
        self.com = self._midpoint()
        self.comv = [0.0, 0.0]
        self.coma = [0.0, 0.0]
        self.phase = 0.0
        if self._moving(self.p.v_start_eps, march):
            # 横移動は進行方向側の足から踏み出す (walk_core と同じ)
            vy = self.v[1]
            if abs(vy) > 1e-6:
                self.sup = RIGHT if vy > 0 else LEFT
            else:
                self.sup = LEFT
            self._start_shift(self._over_support(), 'SHIFT')

    # ------------------------------------------------------------ 重心移動
    def _start_shift(self, target, state):
        self.state = state
        self.c0 = list(self.com)
        self.c1 = [target[0], target[1]]
        d = math.hypot(self.c1[0] - self.c0[0], self.c1[1] - self.c0[1])
        a_lim = self._omega2() * self.q.zmp_tol
        self.dur = max(self.q.t_shift_min, math.sqrt(K_QUINTIC * d / a_lim))
        self.t_local = 0.0
        self.phase = 0.0

    def _tick_shift(self, dt, march):
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
        if self.state == 'STOP':
            self.state = 'IDLE'
            self.phase = 0.0
        elif not self._moving(self.p.v_stop_eps, march) and self._feet_aligned():
            # 足が揃ったまま指令が消えた: 振り出さずに中点へ戻る
            self._start_shift(self._midpoint(), 'STOP')
        else:
            self._start_swing(march)

    # -------------------------------------------------------------- 振り出し
    def _start_swing(self, march):
        p = self.p
        self.state = 'SWING'
        self.step_idx += 1
        self.t_local = 0.0
        self.dur = self.q.t_swing
        self.phase = 0.0
        swing = -self.sup
        self.swing_r0 = list(self.foot[swing])
        self.swing_z = 0.0
        ps = self.foot[self.sup]
        if self._moving(p.v_stop_eps, march):
            self.mode = 'walk'
            self.p_land = [ps[0] + self.v[0] * p.t_step,
                           ps[1] + swing * p.foot_spacing + self.v[1] * p.t_step]
        else:
            self.mode = 'stop'     # 支持足の真横に揃える
            self.p_land = [ps[0], ps[1] + swing * p.foot_spacing]
        self.steps.append(StepRecord(
            step_idx=self.step_idx, t_start=self.t, support=self.sup,
            mode=self.mode, v=tuple(self.v), p_support=tuple(ps),
            p_nom=tuple(self.p_land), p_land=tuple(self.p_land)))

    def _tick_swing(self, dt, march):
        p = self.p
        self.t_local = min(self.t_local + dt, self.dur)
        self.phase = self.t_local / self.dur
        tau = self.phase
        s, _, _ = _s(tau)
        swing = -self.sup
        f = self.foot[swing]
        f[0] = self.swing_r0[0] + s * (self.p_land[0] - self.swing_r0[0])
        f[1] = self.swing_r0[1] + s * (self.p_land[1] - self.swing_r0[1])
        # 高さは walk_core の遊脚と同じ形 (上昇 45%・下降 55%、突き抜けと降下速度の上限)
        if tau < 0.45:
            self.swing_z = self.h_sw * _s(tau / 0.45)[0]
        else:
            u = (tau - 0.45) / 0.55
            su = _s(u)[0]
            z_ref = self.h_sw * (1.0 - su) - p.td_overdrive * su
            self.swing_z = max(z_ref, self.swing_z - p.td_speed_max * dt)
        if self.t_local < self.dur:
            return
        # 着地
        f[0], f[1] = self.p_land
        self.swing_z = 0.0
        self.steps[-1].t_end = self.t
        if self.mode == 'stop' or (
                not self._moving(p.v_stop_eps, march) and self._feet_aligned()):
            # 足が揃った: 重心を中点へ戻して止まる
            self.p_land = None
            self._start_shift(self._midpoint(), 'STOP')
        else:
            self.sup = swing
            self.p_land = None
            self._start_shift(self._over_support(), 'SHIFT')

    # ------------------------------------------------------------------ 出力
    def _outputs(self) -> WalkOutputs:
        w = math.sqrt(self._omega2())
        swinging = self.state == 'SWING'
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
            stopping=(swinging and self.mode == 'stop') or self.state == 'STOP')
