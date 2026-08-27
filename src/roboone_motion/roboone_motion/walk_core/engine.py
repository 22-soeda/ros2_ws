# -*- coding: utf-8 -*-
"""歩行計画エンジン (walk_core)。

docs/ros2_walk_implementation.pdf の §3〜§5 を、次の方針で実装したもの。

* **回転なしの平行移動のみ** (ωz ≡ 0、足ヨー ≡ 0)。文書は支持足座標系 {S_i} で
  計算して歩の境界で式 (3) の乗り換えをするが、回転がないと式 (3) は純平行移動に
  なり、世界座標で積分するのと厳密に等価になる。ここでは世界座標 (初期立位の
  骨盤直下を原点、x 前、y 左) で全量を持ち、支持足位置 p_i を変数として扱う。
  可視化とオドメトリがそのまま取れる利点もある。回転を入れる段になったら、
  この層の外に式 (3) の乗り換えを足す。
* **IMU・状態推定なし** の純フィードフォワード。ξ に入れるのは常に計画値で、
  文書 §3.4 の「ξ の出どころ」スイッチの片側だけを実装した形。式は同じなので、
  推定器ができたら ξ̂ を差し込むだけで踏み出し補正になる。
* 乱数も時計も持たず、update(vx, vy, dt) の入力列だけで決定的に動く (文書 §1.3)。

歩数の添字は文書と同じで、歩 i は支持足 p_i の上で ξ が進み、遊脚が p_{i+1} へ
着地して終わる。

DCM オフセット b の式について (文書式 (6)(7)(8) の一般化):
    歩ごとの着地変位 ℓ_i = p_{i+1} - p_i が 2 歩周期 (ℓ_A, ℓ_B, ℓ_A, ...) の
    定常歩行では、連続条件 b_i e^{ωT} = ℓ_i + b_{i+1} を解くと
        b = (ℓ_A e^{ωT} + ℓ_B) / (e^{2ωT} - 1)        (成分ごと)
    になる。前後 (ℓ_A = ℓ_B = Lx) では文書式 (6) の Lx/(e^{ωT}-1) に、
    左右 (ℓ_A = -ℓ_B) では式 (7) の L/(e^{ωT}+1) に一致する。横移動 (Ly ≠ 0) では
    左右の変位が ±W + Ly と非対称になるため、この一般形をそのまま使う
    (文書式 (8) は Ly ≠ 0 で数 % の近似になる)。
"""

import math
from dataclasses import dataclass
from typing import List, Optional

from .params import GaitParams

# 状態 (文書 §4)。FALL は IMU が要るので今回はない。
IDLE = 'IDLE'
START = 'START'
STEP = 'STEP'
STOP = 'STOP'
ESTOP = 'ESTOP'

LEFT = +1    # 足の符号。+1 左 / -1 右 (y は左が正)
RIGHT = -1


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _quintic(tau: float) -> float:
    """5 次多項式 s(τ) = 10τ³ - 15τ⁴ + 6τ⁵ (式 14)。s(0)=0, s(1)=1, 端点速度加速度 0。"""
    tau = _clamp(tau, 0.0, 1.0)
    return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))


@dataclass
class WalkOutputs:
    """1 周期ぶんの出力。座標は全て世界座標 [m]。"""
    t: float = 0.0
    state: str = IDLE
    step_idx: int = 0
    phase: float = 0.0            # 歩の位相 φ = t_local / T (START では押し出し経過)
    support: int = 0              # +1 左足支持 / -1 右足支持 / 0 両足
    v: tuple = (0.0, 0.0)         # 整形後の指令 (式 1, 2)
    xi: tuple = (0.0, 0.0)        # DCM ξ
    com: tuple = (0.0, 0.0)       # 重心 x_C (水平成分)
    zmp: tuple = (0.0, 0.0)       # ZMP 参照 p
    left_foot: tuple = (0.0, 0.0, 0.0)
    right_foot: tuple = (0.0, 0.0, 0.0)
    pelvis: tuple = (0.0, 0.0, 0.0)   # (x_C, z_c)。骨盤=重心の水平投影とみなす
    # 計画中の 1 歩のパラメータ (STEP/START 中のみ。文書 §2 の中段)
    p_nom: Optional[tuple] = None     # 名目着地点 (式 5)
    p_land: Optional[tuple] = None    # 補正・クランプ後の着地点 (式 10, 11)
    b_next: Optional[tuple] = None    # DCM オフセット (式 8 の一般形)
    xi_eos: Optional[tuple] = None    # 歩の終端の ξ 予測 (式 9)
    clamp_box: Optional[tuple] = None  # (xmin, xmax, ymin, ymax) 世界座標
    locked: bool = False              # φ_lock を過ぎて着地点を凍結したか
    stopping: bool = False            # この歩が足を揃える最後の歩か

    def foot_targets_pelvis(self) -> dict:
        """骨盤水平座標系 {L} での足先目標 (今後の IK 接続用)。"""
        px, py, pz = self.pelvis
        lf, rf = self.left_foot, self.right_foot
        return {
            'left': (lf[0] - px, lf[1] - py, lf[2] - pz),
            'right': (rf[0] - px, rf[1] - py, rf[2] - pz),
        }


@dataclass
class StepRecord:
    """確定した 1 歩のパラメータ (デバッグ・可視化用)。"""
    step_idx: int
    t_start: float
    support: int
    stopping: bool
    v: tuple
    p_support: tuple
    p_nom: tuple = None
    p_land: tuple = None
    b_next: tuple = None
    clamped: bool = False
    t_end: float = None


class WalkEngine:
    """update(vx, vy, dt) を回すだけの決定的な歩行計画器。"""

    def __init__(self, params: Optional[GaitParams] = None):
        self.p = params or GaitParams()
        self.reset()

    # ------------------------------------------------------------------ 初期化
    def reset(self):
        p = self.p
        w2 = p.foot_spacing / 2.0
        self.t = 0.0
        self.state = IDLE
        self.step_idx = 0
        self.v = [0.0, 0.0]               # 整形後の指令
        self.foot = {LEFT: [0.0, +w2], RIGHT: [0.0, -w2]}   # 接地位置 (x, y)
        self.sup = LEFT                   # 支持足 (STEP 中のみ意味を持つ)
        self.xi = [0.0, 0.0]
        self.com = [0.0, 0.0]
        self.zmp = [0.0, 0.0]
        self.phase = 0.0
        self.t_local = 0.0                # 歩 (または押し出し) の経過時間
        self.xi_ini = [0.0, 0.0]          # 歩の始点の ξ (式 12)
        self.swing_r0 = [0.0, 0.0]        # 遊脚の始点 (式 15)
        self.swing_z = 0.0
        self.p_nom = None
        self.p_land = None
        self.b_next = None
        self.xi_eos = None
        self.clamp_box = None
        self.locked = False
        self.stopping = False
        self.steps: List[StepRecord] = []  # 歩の履歴 (可視化用)

    # ------------------------------------------------------------ 指令の整形
    def _shape_cmd(self, vx: float, vy: float, dt: float):
        """式 (1) の飽和+レート制限と、式 (2) を斜め歩きに読み替えた楕円制限。"""
        p = self.p
        vx = _clamp(vx, -p.v_max[0], p.v_max[0])
        vy = _clamp(vy, -p.v_max[1], p.v_max[1])
        # 前進と横移動の同時要求で遊脚の到達域 (文書式 18) を超えないよう楕円で縮める
        s = math.hypot(vx / p.v_max[0], vy / p.v_max[1])
        if s > 1.0:
            vx /= s
            vy /= s
        for k, (v_in, a) in enumerate(zip((vx, vy), p.a_max)):
            self.v[k] += _clamp(v_in - self.v[k], -a * dt, a * dt)

    # ------------------------------------------------- 1 歩のパラメータ (文書 §3)
    def _step_params(self):
        """支持足 self.sup から次の 1 歩の名目着地点と DCM オフセットを出す。

        戻り値: (p_nom, b_here, b_next)。世界座標。
        b_here は現在の支持足での ξ 始点オフセット b_i (START の遷移目標に使う)、
        b_next は次の支持足でのオフセット b_{i+1} (式 10 に使う)。
        """
        p = self.p
        lx = self.v[0] * p.t_step
        ly = self.v[1] * p.t_step
        w = p.foot_spacing
        s_next = -self.sup
        px, py = self.foot[self.sup]
        p_nom = [px + lx, py + s_next * w + ly]           # 式 (5) (Δψ = 0)
        # 2 歩周期の定常解 (モジュール docstring 参照)。
        # l_first = p_i → p_{i+1}、l_second = p_{i+1} → p_{i+2} の変位。
        ewt = p.e_wt
        denom = ewt * ewt - 1.0
        l_first = (lx, s_next * w + ly)
        l_second = (lx, self.sup * w + ly)
        b_here = [(l_first[k] * ewt + l_second[k]) / denom for k in (0, 1)]
        b_next = [(l_second[k] * ewt + l_first[k]) / denom for k in (0, 1)]
        return p_nom, b_here, b_next

    def _clamp_landing(self, p_land, p_nom):
        """式 (11)。クランプ域は名目着地点まわりで、内外は着地脚の側で決まる。"""
        p = self.p
        s_next = -self.sup
        xmin = p_nom[0] - p.step_clamp_x
        xmax = p_nom[0] + p.step_clamp_x
        if s_next == LEFT:      # 左足が着く: 外側 = +y
            ymin = p_nom[1] - p.step_clamp_in
            ymax = p_nom[1] + p.step_clamp_out
        else:                   # 右足が着く: 外側 = -y
            ymin = p_nom[1] - p.step_clamp_out
            ymax = p_nom[1] + p.step_clamp_in
        self.clamp_box = (xmin, xmax, ymin, ymax)
        return [_clamp(p_land[0], xmin, xmax), _clamp(p_land[1], ymin, ymax)]

    def _predict_xi_eos(self):
        """式 (9)。歩の終端の ξ 予測 (計画値なので閉形式で厳密)。"""
        p = self.p
        e = math.exp(p.omega * (p.t_step - self.t_local))
        sx, sy = self.foot[self.sup]
        return [sx + (self.xi[0] - sx) * e, sy + (self.xi[1] - sy) * e]

    def _update_landing(self):
        """式 (9)〜(11)。φ_lock までは毎周期呼んで着地点を更新する。"""
        p = self.p
        p_nom, _, b = self._step_params()
        xi_eos = self._predict_xi_eos()
        # 式 (10): 名目終端 ξ からのずれを着地点で吸収する (計画では START の
        # 過渡と指令変化がここに現れる)
        raw = [p_nom[k] + p.k_dcm * (xi_eos[k] - (p_nom[k] + b[k])) for k in (0, 1)]
        self.p_nom = p_nom
        self.b_next = b
        self.xi_eos = xi_eos
        self.p_land = self._clamp_landing(raw, p_nom)

    def _update_stop_landing(self):
        """停止の最後の歩 (文書 §4.3)。着地点は「終端 ξ が両足の中点になる」位置。

        名目は支持足の真横 p_N = p_{N-1} + (0, s W)。式 (21) は b の形だが、
        計画では終端 ξ が閉形式で出るので、中点条件 m = ξ_eos を直接
        p_N = 2 ξ_eos - p_{N-1} と解いてクランプする方が単純で等価。
        """
        p = self.p
        s_next = -self.sup
        sx, sy = self.foot[self.sup]
        p_nom = [sx, sy + s_next * p.foot_spacing]
        xi_eos = self._predict_xi_eos()
        raw = [2.0 * xi_eos[k] - (sx, sy)[k] for k in (0, 1)]
        self.p_nom = p_nom
        self.b_next = None
        self.xi_eos = xi_eos
        self.p_land = self._clamp_landing(raw, p_nom)

    # ------------------------------------------------------------ 歩の境界処理
    def _enter_step(self):
        """文書 §4.2 の境界処理。呼ぶ前に self.sup を新しい支持足にしておく。"""
        p = self.p
        self.state = STEP
        self.step_idx += 1
        self.phase = 0.0
        self.t_local = 0.0
        self.locked = False
        self.zmp = list(self.foot[self.sup])
        self.xi_ini = list(self.xi)
        swing = -self.sup
        self.swing_r0 = list(self.foot[swing])   # いま床を離れる足の現在位置
        self.swing_z = 0.0
        self.stopping = math.hypot(*self.v) < p.v_stop_eps
        if self.stopping:
            self._update_stop_landing()
            self.locked = True                   # 停止歩は指令で動かさない
        else:
            self._update_landing()
        self.steps.append(StepRecord(
            step_idx=self.step_idx, t_start=self.t, support=self.sup,
            stopping=self.stopping, v=tuple(self.v),
            p_support=tuple(self.foot[self.sup])))

    def _finish_step_record(self):
        if self.steps:
            r = self.steps[-1]
            r.p_nom = tuple(self.p_nom)
            r.p_land = tuple(self.p_land)
            r.b_next = tuple(self.b_next) if self.b_next else None
            # _clamp_landing は飽和時に境界値そのものを返すので、一致 = クランプが効いた
            box = self.clamp_box
            r.clamped = box is not None and (
                self.p_land[0] in (box[0], box[1])
                or self.p_land[1] in (box[2], box[3]))
            r.t_end = self.t

    # ------------------------------------------------------------ 遊脚 (文書 §3.6)
    def _swing_pos(self, dt: float):
        """式 (14)〜(16)。ψ は常に 0 なので式 (17) はない。"""
        p = self.p
        tau = _clamp(self.phase, 0.0, 1.0)
        s = _quintic(tau)
        x = self.swing_r0[0] + s * (self.p_land[0] - self.swing_r0[0])
        y = self.swing_r0[1] + s * (self.p_land[1] - self.swing_r0[1])
        if tau < 0.45:
            z = p.swing_height * _quintic(tau / 0.45)
            self.swing_z = z
        else:
            u = (tau - 0.45) / 0.55
            z_ref = p.swing_height * (1.0 - _quintic(u)) - p.td_overdrive * _quintic(u)
            # 降下速度を td_speed_max で飽和 (実機の衝撃対策。計画でも同じ形にしておく)
            z = max(z_ref, self.swing_z - p.td_speed_max * dt)
            self.swing_z = z
        return x, y, z

    # ---------------------------------------------------------------- DCM 積分
    def _advance_dcm(self, dt: float):
        """式 (12)。ξ は歩の始点からの閉形式、重心はオイラー積分 (文書どおり)。"""
        w = self.p.omega
        e = math.exp(w * self.t_local)
        for k in (0, 1):
            self.xi[k] = self.zmp[k] + (self.xi_ini[k] - self.zmp[k]) * e
            self.com[k] += w * (self.xi[k] - self.com[k]) * dt

    # ------------------------------------------------------------------- 本体
    def update(self, vx_cmd: float, vy_cmd: float, dt: float,
               estop: bool = False) -> WalkOutputs:
        p = self.p
        self.t += dt
        self._shape_cmd(vx_cmd, vy_cmd, dt)

        if estop:
            self.state = ESTOP
        if self.state == ESTOP:
            # 脱力。計画値は凍結し、復帰は reset() (実機では home 技) から。
            return self._outputs()

        if self.state == IDLE:
            self._tick_idle()
        elif self.state == START:
            self._tick_start(dt)
        elif self.state == STEP:
            self._tick_step(dt)
        elif self.state == STOP:
            self._tick_stop(dt)
        return self._outputs()

    # ------------------------------------------------------------------ IDLE
    def _tick_idle(self):
        p = self.p
        mid = self._midpoint()
        self.xi = list(mid)
        self.com = list(mid)
        self.zmp = list(mid)
        self.p_nom = self.p_land = self.b_next = self.xi_eos = None
        self.clamp_box = None
        self.stopping = False
        if math.hypot(*self.v) >= p.v_start_eps:
            self._enter_start()

    def _enter_start(self):
        """文書 §4.1。最初の支持足を決め、反対の足 (最初の遊脚) で ξ を押し出す。

        横移動があるときは進行方向側の足から踏み出す (右へ歩くのに左足から
        出すと 1 歩目が閉じる方向になり狭い内側クランプに当たる)。
        既定 (前後のみ) は文書どおり左支持・右足から。
        """
        vy = self.v[1]
        if abs(vy) > 1e-6:
            self.sup = RIGHT if vy > 0 else LEFT   # 遊脚 = 進行方向側
        else:
            self.sup = LEFT
        self.state = START
        self.t_local = 0.0
        self.phase = 0.0
        self.xi_ini = list(self.xi)
        self.zmp = list(self.foot[-self.sup])      # ZMP は押し出し足 (式 19)
        self.stopping = False

    # ----------------------------------------------------------------- START
    def _tick_start(self, dt: float):
        p = self.p
        self.t_local += dt
        self.phase = self.t_local / p.start_pushoff_max
        self._advance_dcm(dt)
        # 遷移目標: 支持足の上 + 最初の歩の始点オフセット b_1
        # (指令の立ち上がりに追従して毎周期更新する)
        p_nom, b_here, b_next = self._step_params()
        self.p_nom, self.b_next = p_nom, b_next
        self.p_land = None
        self.xi_eos = None
        self.clamp_box = None
        target_y = self.foot[self.sup][1] + b_here[1]
        if self.sup * (self.xi[1] - target_y) >= 0.0:
            self._enter_step()          # ξ が支持足の上に乗った (式 20 の条件版)
        elif self.t_local > p.start_pushoff_max:
            # 押し出し切れず。実機では異常だが、計画では静かに立位へ戻す
            self.state = STOP
        elif math.hypot(*self.v) < p.v_stop_eps:
            self.state = STOP           # 押し出し中に指令が消えた

    # ------------------------------------------------------------------ STEP
    def _tick_step(self, dt: float):
        p = self.p
        self.t_local += dt
        self.phase = self.t_local / p.t_step
        self._advance_dcm(dt)
        if not self.stopping:
            if self.phase < p.swing_lock_phase:
                self._update_landing()
            else:
                self.locked = True
        swing = -self.sup
        sx, sy, sz = self._swing_pos(dt)
        self.foot[swing][0] = sx
        self.foot[swing][1] = sy
        if self.phase >= 1.0:
            self._land(swing)

    def _land(self, swing: int):
        """着地 = 歩の境界。計画では位相 1.0 ちょうどで着く (接地判定は実機で)。"""
        self._finish_step_record()
        self.foot[swing][0] = self.p_land[0]
        self.foot[swing][1] = self.p_land[1]
        self.swing_z = 0.0
        if self.stopping:
            self.state = STOP
            self.p_nom = self.p_land = self.b_next = self.xi_eos = None
            self.clamp_box = None
        else:
            self.sup = swing            # 支持脚の交代
            self._enter_step()

    # ------------------------------------------------------------------ STOP
    def _tick_stop(self, dt: float):
        """両足支持で ξ の位置に ZMP を置いて静止する (文書 §4.3)。

        ξ が支持多角形 (両足中心を結ぶ線分で近似) の外なら、もう 1 歩踏む。
        """
        p = self.p
        proj, dist = self._project_between_feet(self.xi)
        if dist > p.stop_outside_eps:
            # 収束できない。ξ に近い側を支持足にしてもう 1 歩 (計画では通常来ない)
            self.sup = self._nearer_foot(self.xi)
            self.v = [0.0, 0.0]
            self._enter_step()
            return
        self.zmp = proj                  # ZMP を ξ に置く → ξ は動かない
        self.xi_ini = list(self.xi)
        self.t_local = 0.0
        self._advance_dcm(dt)            # ξ は不動、重心だけ ξ へ収束
        self.phase = 0.0
        if (abs(self.xi[0] - self.com[0]) < p.settle_eps
                and abs(self.xi[1] - self.com[1]) < p.settle_eps):
            self.state = IDLE

    # ------------------------------------------------------------------ 補助
    def _midpoint(self):
        return [(self.foot[LEFT][0] + self.foot[RIGHT][0]) / 2.0,
                (self.foot[LEFT][1] + self.foot[RIGHT][1]) / 2.0]

    def _nearer_foot(self, pt) -> int:
        dl = math.hypot(pt[0] - self.foot[LEFT][0], pt[1] - self.foot[LEFT][1])
        dr = math.hypot(pt[0] - self.foot[RIGHT][0], pt[1] - self.foot[RIGHT][1])
        return LEFT if dl <= dr else RIGHT

    def _project_between_feet(self, pt):
        """点を両足中心を結ぶ線分へ射影する。戻り値 (射影点, 距離)。"""
        a = self.foot[LEFT]
        b = self.foot[RIGHT]
        abx, aby = b[0] - a[0], b[1] - a[1]
        den = abx * abx + aby * aby
        u = 0.0 if den == 0 else _clamp(
            ((pt[0] - a[0]) * abx + (pt[1] - a[1]) * aby) / den, 0.0, 1.0)
        proj = [a[0] + u * abx, a[1] + u * aby]
        d = math.hypot(pt[0] - proj[0], pt[1] - proj[1])
        return proj, d

    def _outputs(self) -> WalkOutputs:
        in_step = self.state == STEP
        swing = -self.sup
        lf = (self.foot[LEFT][0], self.foot[LEFT][1],
              self.swing_z if in_step and swing == LEFT else 0.0)
        rf = (self.foot[RIGHT][0], self.foot[RIGHT][1],
              self.swing_z if in_step and swing == RIGHT else 0.0)
        return WalkOutputs(
            t=self.t, state=self.state, step_idx=self.step_idx,
            phase=self.phase,
            support=(self.sup if in_step else 0),
            v=tuple(self.v), xi=tuple(self.xi), com=tuple(self.com),
            zmp=tuple(self.zmp), left_foot=lf, right_foot=rf,
            pelvis=(self.com[0], self.com[1], self.p.z_c),
            p_nom=tuple(self.p_nom) if self.p_nom else None,
            p_land=tuple(self.p_land) if self.p_land else None,
            b_next=tuple(self.b_next) if self.b_next else None,
            xi_eos=tuple(self.xi_eos) if self.xi_eos else None,
            clamp_box=self.clamp_box,
            locked=self.locked, stopping=self.stopping)
