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

両足支持 (ds_time = Td > 0) について:
    各歩の頭に Td の両足支持を置き、ZMP を前の支持足 p_{i-1} から新しい支持足 p_i へ
    直線で移す (速度 ṗ = l_i/Td、l_i = p_i - p_{i-1})。この間の ξ は閉形式で
        ξ(t) = p(t) + ṗ/ω + (ξ_0 - p_{i-1} - ṗ/ω) e^{ωt}
    その後の単脚支持 Ts (= t_step) は従来と同じく ZMP を p_i に置く。
    歩の頭の ZMP から見た ξ のずれを c_i = ξ_0 - p_{i-1} とおくと、1 歩で
        c_{i+1} = ξ_end - p_i = E c_i - K l_i
        E = e^{ω(Td+Ts)},  K = κ e^{ωTs},  κ = (e^{ωTd} - 1)/(ωTd)
    2 歩周期の定常解は c_i = K (E l_i + l_{i+1}) / (E² - 1)。Td → 0 で κ → 1、
    c_i - l_i が上の b に戻る。
    着地点 p_{i+1} (= l_{i+1}) は、次の歩の終わりで定常解に戻るように選ぶ:
        E c_{i+1} - K l_{i+1} = c_{i+2}(名目)
        ⇔  p_{i+1} = p_nom + (e^{ωTd}/κ)(ξ_eos - p_i - c_{i+1}(名目))
    (1 周期で戻る。Td → 0 で式 (10) に一致)。歩き出しは、押し出しで ξ を
    両足の中点 m から c_1 = (c_2(名目) + K (p_1 - m))/E だけ進めてから、中点から最初の
    支持足への両足支持に入る。停止は、最後の歩の後にもう一度両足支持を置いて ZMP を
    両足の中点へ移し、そこで ξ が中点に止まるように最後の 2 歩の着地点を選ぶ。
    **ds_time = 0 のときは従来の経路をそのまま通る** (数値は変わらない)。
"""

from dataclasses import dataclass
import math
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


def check_swing_landing(p: GaitParams) -> dict:
    """遊脚が計画どおり床に着くかを、実際の軌道で調べる。

    降下は td_speed_max で飽和するので、**狙った swing_ratio がそのまま両足支持に
    なるわけではない**。式で近似せず、_swing_pos と同じ漸化式を loop_hz で回す
    （あちらを変えたらここも合わせること）。C++ 版は walk_engine.hpp の
    checkSwingLanding()。
    """
    dt = 1.0 / (p.loop_hz if p.loop_hz > 0.0 else 200.0)
    sr = p.swing_ratio if p.swing_ratio > 1e-6 else 1.0
    r = {'lands': False, 'touch_phase': 1.0, 'double_support': 0.0,
         'saturated': False, 'z_end': 0.0,
         'v_need': (p.swing_height + p.td_overdrive) / (0.55 * sr * p.t_step)}
    z = zp = t = 0.0
    while t < p.t_step - 1e-12:
        t += dt
        phase = t / p.t_step
        tau = _clamp(phase / sr, 0.0, 1.0)
        if tau < 0.45:
            zref = p.swing_height * _quintic(tau / 0.45)
        else:
            u = (tau - 0.45) / 0.55
            zref = p.swing_height * (1.0 - _quintic(u)) - p.td_overdrive * _quintic(u)
        zsat = zp - p.td_speed_max * dt
        z = zref if zref > zsat else zsat
        if zsat > zref + 1e-12:
            r['saturated'] = True
        if not r['lands'] and z <= 0.0:
            r['lands'] = True
            r['touch_phase'] = phase
            r['double_support'] = 1.0 - phase
        zp = z
    r['z_end'] = z
    return r


@dataclass
class WalkOutputs:
    """1 周期ぶんの出力。座標は全て世界座標 [m]。"""

    t: float = 0.0
    state: str = IDLE
    step_idx: int = 0
    # 歩の位相 φ = t_local / T (START では押し出し経過。両足支持の間は 0。単脚支持の中で測る)
    phase: float = 0.0
    support: int = 0              # +1 左足支持 / -1 右足支持 / 0 両足
    double_support: bool = False  # 歩の頭 (または停止) の両足支持で ZMP を移している最中か
    ds_elapsed: float = 0.0       # その両足支持の経過時間 [s] (両足支持でなければ 0)
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
    # DCM オフセット (式 8 の一般形)。ds_time > 0 では、いまの支持足から見た
    # 歩の終わりの ξ の名目のずれ c_{i+1} (停止準備歩では最後の歩の終わりの狙い)
    b_next: Optional[tuple] = None
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
    mode: str                 # 'walk' / 'prep' (停止準備) / 'stop' (足を揃える)
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
        self.stopping = False             # いまの歩が足を揃える最後の歩か
        self.stop_prep = False            # 次の歩で足を揃える (この歩は準備歩)
        # 両足支持 (ds_time > 0)。STEP の頭と、STOP の頭 (最後の両足支持) で使う
        self.in_ds = False
        self.ds_t = 0.0                   # 両足支持の経過時間
        self.ds_from = [0.0, 0.0]         # ZMP の出発点
        self.ds_rate = [0.0, 0.0]         # ZMP の速度 ṗ
        self.start_mid = [0.0, 0.0]       # 歩き出しの両足の中点
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

    def _step_params_ds(self):
        """ds_time > 0 版。戻り値: (p_nom, c_next, c_after)。

        c_next は歩の終わりの ξ の名目のずれ (いまの支持足 p_i から見た c_{i+1})、
        c_after はその次の歩の終わりの名目 (p_{i+1} から見た c_{i+2})。
        """
        p = self.p
        lx = self.v[0] * p.t_step
        ly = self.v[1] * p.t_step
        w = p.foot_spacing
        s_next = -self.sup
        px, py = self.foot[self.sup]
        p_nom = [px + lx, py + s_next * w + ly]
        _, _, e, k = p.ds_consts()
        denom = e * e - 1.0
        l_first = (lx, s_next * w + ly)     # p_i -> p_{i+1}
        l_second = (lx, self.sup * w + ly)  # p_{i+1} -> p_{i+2}
        c_next = [k * (e * l_first[i] + l_second[i]) / denom for i in (0, 1)]
        c_after = [k * (e * l_second[i] + l_first[i]) / denom for i in (0, 1)]
        return p_nom, c_next, c_after

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
        sx, sy = self.foot[self.sup]
        if self.in_ds:
            # 両足支持の残りを閉形式で進めてから、単脚支持 Ts を丸ごと進める
            w = p.omega
            e_rem = math.exp(w * (p.ds_time - self.ds_t))
            out = []
            for k, s in enumerate((sx, sy)):
                v = self.ds_rate[k] / w
                rel_d = v + (self.xi[k] - self.zmp[k] - v) * e_rem   # ξ(Td) - p_i
                out.append(s + rel_d * p.e_wt)
            return out
        e = math.exp(p.omega * (p.t_step - self.t_local))
        return [sx + (self.xi[0] - sx) * e, sy + (self.xi[1] - sy) * e]

    def _update_landing(self):
        """式 (9)〜(11)。φ_lock までは毎周期呼んで着地点を更新する。"""
        p = self.p
        if p.ds_time > 0.0:
            # 次の歩の終わりで定常解に戻る着地点 (モジュール docstring)。
            # ずれの吸収に要るずらしは e^{ωTd}/κ 倍になる
            p_nom, c_next, _ = self._step_params_ds()
            xi_eos = self._predict_xi_eos()
            ed, kappa, _, _ = p.ds_consts()
            s = self.foot[self.sup]
            g = p.k_dcm * ed / kappa
            raw = [p_nom[k] + g * (xi_eos[k] - s[k] - c_next[k]) for k in (0, 1)]
            self.p_nom = p_nom
            self.b_next = c_next
            self.xi_eos = xi_eos
            self.p_land = self._clamp_landing(raw, p_nom)
            return
        p_nom, _, b = self._step_params()
        xi_eos = self._predict_xi_eos()
        # 式 (10): 名目終端 ξ からのずれを着地点で吸収する (計画では START の
        # 過渡と指令変化がここに現れる)
        raw = [p_nom[k] + p.k_dcm * (xi_eos[k] - (p_nom[k] + b[k])) for k in (0, 1)]
        self.p_nom = p_nom
        self.b_next = b
        self.xi_eos = xi_eos
        self.p_land = self._clamp_landing(raw, p_nom)

    def _update_prep_landing(self):
        """停止準備歩 (歩 N-1 に入る前の歩 N-2)。文書 §4.3 式 (21)。

        停止を判断した時点で ξ は歩行の始点オフセットを既に持っていて、
        その歩の終端 ξ は変えられない (ξ_eos は支持足と ξ_ini だけで決まる)。
        変えられるのは着地点なので、次の歩の始点オフセットが式 (21) の
        b_{N-1} = (m_N - p_{N-1}) e^{-ωT} = (0, s_N W/2) e^{-ωT}
        になるよう p_{N-1} = ξ_eos - b_{N-1} と置く。s_N (最後に揃える足の側) は
        いまの支持足の側に等しい。
        """
        p = self.p
        s_next = -self.sup
        sx, sy = self.foot[self.sup]
        p_nom = [sx, sy + s_next * p.foot_spacing]   # v=0 の名目 (真横)
        xi_eos = self._predict_xi_eos()
        if p.ds_time > 0.0:
            # 最後の歩 (支持足 p_{N-1}) の終わりに、ξ が p_{N-1} から
            # d = κ/(2 e^{ωTd}) · (p_N - p_{N-1}) にいれば、最後の両足支持で ZMP を中点へ
            # 移したときに ξ も中点で止まる。その d に最後の歩で着くよう p_{N-1} を選ぶ
            ed, kappa, e, k = p.ds_consts()
            d = (0.0, kappa / (2.0 * ed) * self.sup * p.foot_spacing)
            c = (xi_eos[0] - sx, xi_eos[1] - sy)
            raw = [(sx, sy)[i] + (e * c[i] - d[i]) / k for i in (0, 1)]
            self.p_nom = p_nom
            self.b_next = list(d)
            self.xi_eos = xi_eos
            self.p_land = self._clamp_landing(raw, p_nom)
            return
        b_stop = (0.0, self.sup * (p.foot_spacing / 2.0) / p.e_wt)
        raw = [xi_eos[k] - b_stop[k] for k in (0, 1)]
        self.p_nom = p_nom
        self.b_next = list(b_stop)
        self.xi_eos = xi_eos
        self.p_land = self._clamp_landing(raw, p_nom)

    def _update_stop_landing(self):
        """停止の最後の歩 (文書 §4.3 の歩 N-1)。着地点は「終端 ξ が両足の中点」。

        名目は支持足の真横 p_N = p_{N-1} + (0, s W)。準備歩が式 (21) どおりに
        踏めていれば ξ_eos はちょうど中点に来るので、中点条件 m = ξ_eos を
        p_N = 2 ξ_eos - p_{N-1} と解いてクランプする (補正が要らなければ名目に一致)。
        """
        p = self.p
        s_next = -self.sup
        sx, sy = self.foot[self.sup]
        p_nom = [sx, sy + s_next * p.foot_spacing]
        xi_eos = self._predict_xi_eos()
        if p.ds_time > 0.0:
            # 最後の両足支持で中点 m へ移す ZMP に対し、ξ が m で止まる条件
            # ξ_eos - p_{N-1} = κ/e^{ωTd} · (m - p_{N-1}) を p_N について解く
            ed, kappa, _, _ = p.ds_consts()
            g = 2.0 * ed / kappa
            raw = [(sx, sy)[k] + g * (xi_eos[k] - (sx, sy)[k]) for k in (0, 1)]
        else:
            raw = [2.0 * xi_eos[k] - (sx, sy)[k] for k in (0, 1)]
        self.p_nom = p_nom
        self.b_next = None
        self.xi_eos = xi_eos
        self.p_land = self._clamp_landing(raw, p_nom)

    # ------------------------------------------------------------ 歩の境界処理
    def _enter_step(self, ds_from=None):
        """文書 §4.2 の境界処理。呼ぶ前に self.sup を新しい支持足にしておく。

        ds_time > 0 なら、ZMP を ds_from (省略時はいまの ZMP = 前の支持足) から
        新しい支持足へ移す両足支持から始める。
        """
        p = self.p
        self.state = STEP
        self.step_idx += 1
        self.phase = 0.0
        self.t_local = 0.0
        self.locked = False
        self.in_ds = p.ds_time > 0.0
        if self.in_ds:
            self._start_ds(self.zmp if ds_from is None else ds_from, self.foot[self.sup])
        else:
            self.zmp = list(self.foot[self.sup])
        self.xi_ini = list(self.xi)
        swing = -self.sup
        self.swing_r0 = list(self.foot[swing])   # いま床を離れる足の現在位置
        self.swing_z = 0.0
        v_small = math.hypot(*self.v) < p.v_stop_eps
        if self.stop_prep:
            # 準備歩の次 = 足を揃える最後の歩。ここまで来たら指令が復活しても
            # 完了させる (途中復帰は ξ の整合が崩れて発散するため。歩き直しは
            # 停止後に START からやり直す)。
            mode = 'stop'
            self.stopping = True
            self.stop_prep = False
            self._update_stop_landing()
            self.locked = True
        elif v_small:
            mode = 'prep'
            self.stopping = False
            self.stop_prep = True
            self._update_prep_landing()
            self.locked = True                   # ξ_eos も b も歩の中で不変
        else:
            mode = 'walk'
            self.stopping = False
            self._update_landing()
        self.steps.append(StepRecord(
            step_idx=self.step_idx, t_start=self.t, support=self.sup,
            mode=mode, v=tuple(self.v),
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
        # swing_ratio < 1 なら遊脚は歩の前半だけで軌道を終え、残りは着地点に置かれた
        # まま止まる（= 両足接地）。**ZMP は _enter_step() で支持足に固定されて歩の
        # 間ずっと動かない**ので、ここを変えても DCM の伝播も b の閉形式も変わらない。
        tau = _clamp(self.phase / p.swing_ratio, 0.0, 1.0)
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
    def _start_ds(self, frm, to):
        """ZMP を frm から to へ ds_time で直線に移す両足支持を始める。"""
        td = self.p.ds_time
        self.in_ds = True
        self.ds_t = 0.0
        self.phase = 0.0
        self.ds_from = list(frm)
        self.ds_rate = [(to[k] - frm[k]) / td for k in (0, 1)]
        self.zmp = list(frm)
        self.xi_ini = list(self.xi)

    def _advance_ds(self, dt: float):
        """両足支持の 1 周期。ξ は始点からの閉形式、重心はオイラー積分。

        終わり (ds_t = Td) は閉形式でちょうどに取る (歩の境界と同じ増幅対策)。
        戻り値: 両足支持が終わったか。
        """
        p = self.p
        w = p.omega
        self.ds_t = min(self.ds_t + dt, p.ds_time)
        e = math.exp(w * self.ds_t)
        for k in (0, 1):
            v = self.ds_rate[k] / w
            self.zmp[k] = self.ds_from[k] + self.ds_rate[k] * self.ds_t
            self.xi[k] = self.zmp[k] + v + (self.xi_ini[k] - self.ds_from[k] - v) * e
            self.com[k] += w * (self.xi[k] - self.com[k]) * dt
        return self.ds_t >= p.ds_time

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
        self.start_mid = self._midpoint()
        self.zmp = list(self.foot[-self.sup])      # ZMP は押し出し足 (式 19)
        self.stopping = False
        self.stop_prep = False

    # ----------------------------------------------------------------- START
    def _tick_start(self, dt: float):
        p = self.p
        self.t_local += dt
        self.phase = self.t_local / p.start_pushoff_max
        self._advance_dcm(dt)
        # 遷移目標: 支持足の上 + 最初の歩の始点オフセット b_1
        # (指令の立ち上がりに追従して毎周期更新する)
        if p.ds_time > 0.0:
            # 最初の歩は中点 m から支持足への両足支持で始まる。その歩の終わりで定常解に
            # 乗るには、両足支持の頭で ξ が m から c_1 = (c_2 + K (p_1 - m))/E にいればよい
            p_nom, c_next, _ = self._step_params_ds()
            _, _, e, k = p.ds_consts()
            m = self.start_mid
            ps = self.foot[self.sup]
            c1 = [(c_next[i] + k * (ps[i] - m[i])) / e for i in (0, 1)]
            self.p_nom, self.b_next = p_nom, c_next
            target_y = m[1] + c1[1]
        else:
            p_nom, b_here, b_next = self._step_params()
            self.p_nom, self.b_next = p_nom, b_next
            target_y = self.foot[self.sup][1] + b_here[1]
        self.p_land = None
        self.xi_eos = None
        self.clamp_box = None
        if self.sup * (self.xi[1] - target_y) >= 0.0:
            # ξ が支持足の上に乗った (式 20 の条件版)。
            # 5 ms 刻みのままだと交差の行き過ぎが最大 ~3 mm 出て、それが次の歩で
            # e^{ωT} ≈ 22.9 倍に増幅される。交差時刻 t* を閉形式で解き、ξ を
            # 交差点ちょうどに置いてから歩に入る (時間の ≤5 ms のずれは無視する)。
            zy = self.zmp[1]
            y0 = self.xi_ini[1]
            if abs(y0 - zy) > 1e-12 and (target_y - zy) / (y0 - zy) > 0.0:
                e_star = (target_y - zy) / (y0 - zy)
                self.xi[0] = self.zmp[0] + (self.xi_ini[0] - self.zmp[0]) * e_star
                self.xi[1] = target_y
            self._enter_step(ds_from=self.start_mid if p.ds_time > 0.0 else None)
        elif self.t_local > p.start_pushoff_max:
            # 押し出し切れず。実機では異常だが、計画では静かに立位へ戻す
            self.state = STOP
        elif math.hypot(*self.v) < p.v_stop_eps:
            self.state = STOP           # 押し出し中に指令が消えた

    # ------------------------------------------------------------------ STEP
    def _tick_step(self, dt: float):
        p = self.p
        if self.in_ds:
            # 両足支持: 足は動かさず ZMP だけ移す。着地点は先に更新しておく
            # (ξ の予測は両足支持の残りを含む)。位相は単脚支持の中で測る
            self.phase = 0.0
            done = self._advance_ds(dt)
            if not self.locked:
                self._update_landing()
            if done:
                self.in_ds = False
                self.zmp = list(self.foot[self.sup])
                self.xi_ini = list(self.xi)
                self.t_local = 0.0
            return
        # 歩の境界も閉形式で T ちょうどに取る (START の交差と同じ増幅対策)。
        # 残り時間 ≤5 ms の切り捨ては位相にだけ効き、ξ の整合には効かない。
        self.t_local = min(self.t_local + dt, p.t_step)
        self.phase = self.t_local / p.t_step
        self._advance_dcm(dt)
        if not self.locked:
            if self.phase < p.swing_lock_phase:
                self._update_landing()
            else:
                self.locked = True
        swing = -self.sup
        sx, sy, sz = self._swing_pos(dt)
        self.foot[swing][0] = sx
        self.foot[swing][1] = sy
        if self.t_local >= p.t_step:
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
            if self.p.ds_time > 0.0:
                # 最後の両足支持: ZMP を支持足から両足の中点へ移す
                self._start_ds(self.foot[self.sup], self._midpoint())
        else:
            self.sup = swing            # 支持脚の交代
            self._enter_step()

    # ------------------------------------------------------------------ STOP
    def _tick_stop(self, dt: float):
        """両足支持で ξ の位置に ZMP を置いて静止する (文書 §4.3)。

        ξ が支持多角形 (両足中心を結ぶ線分で近似) の外なら、もう 1 歩踏む。
        """
        p = self.p
        if self.in_ds:
            self.phase = 0.0
            if self._advance_ds(dt):
                self.in_ds = False
            return
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
        in_ds = self.in_ds and self.state in (STEP, STOP)
        swing = -self.sup
        lf = (self.foot[LEFT][0], self.foot[LEFT][1],
              self.swing_z if in_step and swing == LEFT else 0.0)
        rf = (self.foot[RIGHT][0], self.foot[RIGHT][1],
              self.swing_z if in_step and swing == RIGHT else 0.0)
        return WalkOutputs(
            t=self.t, state=self.state, step_idx=self.step_idx,
            phase=self.phase,
            support=(self.sup if in_step and not in_ds else 0),
            double_support=in_ds,
            ds_elapsed=(self.ds_t if in_ds else 0.0),
            v=tuple(self.v), xi=tuple(self.xi), com=tuple(self.com),
            zmp=tuple(self.zmp), left_foot=lf, right_foot=rf,
            pelvis=(self.com[0], self.com[1], self.p.z_c),
            p_nom=tuple(self.p_nom) if self.p_nom else None,
            p_land=tuple(self.p_land) if self.p_land else None,
            b_next=tuple(self.b_next) if self.b_next else None,
            xi_eos=tuple(self.xi_eos) if self.xi_eos else None,
            clamp_box=self.clamp_box,
            locked=self.locked, stopping=self.stopping or self.stop_prep)
