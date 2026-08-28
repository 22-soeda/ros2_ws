#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""膝 4 節リンクのモータ角 θ2 <-> 膝関節角 θ4 の双方向変換。

``docs/膝4節リンク導出.tex`` の閉形式をそのまま実装したもの。
コード中の (KN-n) は同文書の式番号を指す。脚 IK が出す膝関節角をサーボ指令に直す層で、
観測側（サーボの実測角 -> 膝関節角）も同じモジュールが賄う。

    脚 IK -> (θ1..θ6) -> { J1-J3 は直結 / J4 はここ / J5-J6 は足首パラレルリンク }
                      -> サーボ指令
    観測は逆順。

機構と座標系
    大腿に載せたモータがクランクとカプラを介して、下腿に生えたレバーを押し引きする
    平面 4 節リンク。矢状面に載るので 2 次元で閉じる。

        O4 = (0, 0)                       膝軸。原点
        O2 = (r1, 0)                      モータ軸
        A  = O2 + r2·(cos θ2, sin θ2)     クランク先端のピン
        B  = O4 + r4·(cos θ4, sin θ4)     ロッカー側のピン
        拘束: |B − A| = r3                                              (KN-3)

    角度はすべて +x から反時計回り。θ2 = モータ角、θ4 = ロッカー角、
    θ3 = カプラ角は A と B が決まれば従属に決まるので独立変数ではない。

なぜ反復が要らないか
    拘束 Λ := |B − A|² − r3² は (cos θ2, sin θ2) についても (cos θ4, sin θ4) について
    も 1 次である。未知数は 1 つしかないので、片方が既知になれば残る 1 組が直接解ける。
    順変換も逆変換も「三角形 O4–A–B を残った 1 辺の側から閉じる」同じ形になる
    ((KN-5) と (KN-7))。ニュートン法も数値ソルバも使わない。
    半角正接（Weierstrass）置換で 2 次式に落とす手もあるが θ = π 近傍で破綻するので、
    atan2 と arccos のまま扱う。

    足首 (J5/J6) は 2 自由度・2 ループなので事情が違う（逆変換は閉形式、順変換は 8 次
    式で 1 変数ニュートン法）。膝と同じ扱いはできない。``足首パラレルリンク導出.tex``。

寸法・枝・原点はすべて ``knee_config.py`` にある。**暫定値が混じっている**ので、
確定したらあちらだけを書き換える（この本体は触らなくてよい）。

単位は内部 rad・mm で統一。度との変換は入出力の境界（*_deg のラッパ）だけで行う。

    python3 scripts/knee_fourbar.py          # 自己検算と導出 §5 の参照値の再現
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass

try:
    import knee_config as cfg
except ModuleNotFoundError:          # 別ディレクトリから import された場合
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import knee_config as cfg

__all__ = [
    "KneeFourBar", "KneePose", "Unreachable", "DeadPoint",
    "knee_fourbar", "branches_from_pose", "cfg",
]

#: arccos の引数がこの分だけ 1 を超えるのは丸めとみなしてクランプする。
#: これを超えたら本当に三角形が閉じていないので Unreachable にする。
_COS_TOL = 1e-12

#: 長さがこれ以下なら atan2 の向きが決まらないものとして退化扱いにする [mm]。
_LEN_TOL = 1e-12


class Unreachable(ValueError):
    """三角形が閉じない姿勢（可動域の外）。

    歩行中に出るのは軌道生成が可動域を超えたときなので、握り潰さずに記録する。
    ``leg_ik.Unreachable`` と役割は同じ。どちらも ``ValueError`` の派生なので、
    変換層をまとめて捕まえるなら ``except ValueError`` でよい。
    """


class DeadPoint(ValueError):
    """死点。2 交点が重なっていて速度の写像が定義できない姿勢。"""


# --------------------------------------------------------------------------
# 小道具
# --------------------------------------------------------------------------
def _wrap_2pi(a: float) -> float:
    """角を [0, 2π) に畳む。

    atan2 の素の戻り値は (−π, π] なので、そのまま使うと θ2 のスイープに対して θ4 が
    2π 跳ぶ。既存の MATLAB 実装が φ を [0, 2π) に揃えているのに合わせる。ここを
    合わせないと θ4 の系列が不連続になり、伝達比の符号判定や可動域チェックが壊れる。
    """
    a = math.fmod(a, 2.0 * math.pi)
    return a + 2.0 * math.pi if a < 0.0 else a


def _wrap_pi(a: float) -> float:
    """角を (−π, π] に畳む。差を取るときに使う。"""
    return math.atan2(math.sin(a), math.cos(a))


def _acos_checked(x: float, what: str) -> float:
    """arccos。丸めぶんだけクランプし、本当に外れていたら Unreachable。

    クランプせずに渡すと |x| がわずかに 1 を超えたときに黙って NaN が返る。
    """
    if not math.isfinite(x):
        raise Unreachable(f"{what}: arccos の引数が有限でない ({x})")
    if abs(x) > 1.0 + _COS_TOL:
        raise Unreachable(f"{what}: |{x:.12g}| > 1 なので三角形が閉じない")
    return math.acos(max(-1.0, min(1.0, x)))


def _cross(u: tuple[float, float], v: tuple[float, float]) -> float:
    """2 次元の外積 u_x·v_y − u_y·v_x。"""
    return u[0] * v[1] - u[1] * v[0]


# --------------------------------------------------------------------------
# 姿勢
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class KneePose:
    """4 節リンクの 1 姿勢。角は rad、長さは mm。"""

    theta2: float                    #: クランク角（モータ側）
    theta3: float                    #: カプラ角（従属）
    theta4: float                    #: ロッカー角（膝ジョイント側）
    A: tuple[float, float]           #: クランク先端のピン
    B: tuple[float, float]           #: ロッカー側のピン

    @property
    def coupler(self) -> tuple[float, float]:
        """d = B − A。カプラのベクトル。"""
        return (self.B[0] - self.A[0], self.B[1] - self.A[1])

    def loop_residual(self, r3: float) -> float:
        """|A + r3·e(θ3) − B|。閉ループが閉じているかの残差 [mm]。"""
        return math.hypot(self.A[0] + r3 * math.cos(self.theta3) - self.B[0],
                          self.A[1] + r3 * math.sin(self.theta3) - self.B[1])


# --------------------------------------------------------------------------
# 本体
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class KneeFourBar:
    """膝 4 節リンク。既定値は ``knee_config.py``。

    4 本のリンク長は必ず引数か設定値で渡る。本機はたまたま r1 = r4 = 20 だが、
    そこに乗った簡約（|w| = √(2r1²(1 − cos θ4)) や atan2(w) = (θ4 + π)/2 など）は
    使っていない。r4 を 26 mm にするだけで前者は 5.6 mm、後者は 3.3° ずれるうえ、
    エラーも警告も出ないため。受け入れテスト 6 がこの検査にあたる。
    """

    r1: float = cfg.R1               #: 地節（大腿）    O4 -> O2
    r2: float = cfg.R2               #: クランク（入力）
    r3: float = cfg.R3               #: カプラ
    r4: float = cfg.R4               #: ロッカー（出力）
    beta: int = cfg.BETA             #: 順変換 (KN-5) の枝 ±1
    eps: int = cfg.EPS               #: 逆変換 (KN-7) の枝 ±1
    #: θ4_rocker = sigma_joint·θ4_joint + theta4_zero   [rad]（暫定値）
    theta4_zero: float = math.radians(cfg.THETA4_ZERO_DEG)
    sigma_joint: int = cfg.SIGMA_JOINT
    #: motor_cmd = phi0 + sigma_motor·gear·θ2           [rad]（暫定値）
    #: この 3 つだけは左右で違う（膝サーボは左右とも ID 4 で、原点は取り付けごと）。
    #: 既定は右脚。左脚は knee_fourbar("left") で作る。
    phi0: float = math.radians(cfg.PHI0_DEG["right"])
    sigma_motor: int = cfg.SIGMA_MOTOR["right"]
    gear: float = cfg.GEAR["right"]

    def __post_init__(self) -> None:
        for name in ("r2", "r3", "r4"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} は正でなければならない")
        if self.r1 < 0.0:
            raise ValueError("r1 は 0 以上（0 はモータ軸が膝軸と同軸の退化ケース）")
        for name in ("beta", "eps", "sigma_joint", "sigma_motor"):
            if getattr(self, name) not in (+1, -1):
                raise ValueError(f"{name} は ±1")
        if self.gear == 0.0:
            raise ValueError("gear は 0 にできない")

    # -- 幾何の小道具 ------------------------------------------------------
    @property
    def O2(self) -> tuple[float, float]:
        """モータ軸の位置。"""
        return (self.r1, 0.0)

    def crank_pin(self, theta2: float) -> tuple[float, float]:
        """A = O2 + r2·e(θ2)。"""
        return (self.r1 + self.r2 * math.cos(theta2), self.r2 * math.sin(theta2))

    def rocker_pin(self, theta4: float) -> tuple[float, float]:
        """B = O4 + r4·e(θ4)。"""
        return (self.r4 * math.cos(theta4), self.r4 * math.sin(theta4))

    # -- 順変換 θ2 -> θ4 ---------------------------------------------------
    def fk(self, theta2: float) -> KneePose:
        """モータ角 -> 膝ロッカー角 (KN-5)。反復なしの閉形式。

        A が定点になるので、|B|² = r4² が θ4 に依らないことを使って

            Λ = 2(G − E·cos θ4 − F·sin θ4),
            E = r4·A_x, F = r4·A_y, G = (r4² + d² − r3²)/2,  d = |A|

        E² + F² = r4²d² なので、

            θ4 = atan2(A_y, A_x) + β·arccos((r4² + d² − r3²)/(2·r4·d))

        幾何としては O4 中心・半径 r4 の円と A 中心・半径 r3 の円の交わりを求めている。
        """
        A = self.crank_pin(theta2)
        d = math.hypot(A[0], A[1])
        if d < _LEN_TOL:
            raise Unreachable("d = |A| = 0: クランク先端が膝軸に重なっている")
        # arccos の引数は E, F を経由せずに書ける（E² + F² = r4²d² より）
        c = (self.r4 * self.r4 + d * d - self.r3 * self.r3) / (2.0 * self.r4 * d)
        theta4 = _wrap_2pi(math.atan2(A[1], A[0])) + self.beta * _acos_checked(
            c, f"順変換 θ2 = {math.degrees(theta2):.3f} deg")
        B = self.rocker_pin(theta4)
        theta3 = math.atan2(B[1] - A[1], B[0] - A[0])
        return KneePose(theta2, theta3, theta4, A, B)

    # -- 逆変換 θ4 -> θ2 ---------------------------------------------------
    def ik(self, theta4: float) -> KneePose:
        """膝ロッカー角 -> モータ角 (KN-7)。反復なしの閉形式。

        B が定点になる。w := B − O2 と置いて

            Λ = 2·r2·(S − P'·cos θ2 − Q'·sin θ2),
            P' = w_x, Q' = w_y, S = (|w|² + r2² − r3²)/(2·r2)

        P'² + Q'² = |w|² なので、

            θ2 = atan2(w_y, w_x) + ε·arccos((|w|² + r2² − r3²)/(2·r2·|w|))

        (KN-5) と同じ形。三角形 O4–A–B を A の側から閉じるか B の側から閉じるかの
        違いしかない。ここでは r1 = r4 に依る簡約を使っていないので、r1 と r4 は
        独立に変えてよい。
        """
        B = self.rocker_pin(theta4)
        w = (B[0] - self.r1, B[1])
        wn = math.hypot(w[0], w[1])
        if wn < _LEN_TOL:
            raise Unreachable("|w| = 0: ロッカー先端がモータ軸に重なっている")
        s = (wn * wn + self.r2 * self.r2 - self.r3 * self.r3) / (2.0 * self.r2 * wn)
        theta2 = _wrap_2pi(math.atan2(w[1], w[0])) + self.eps * _acos_checked(
            s, f"逆変換 θ4 = {math.degrees(theta4):.3f} deg")
        A = self.crank_pin(theta2)
        theta3 = math.atan2(B[1] - A[1], B[0] - A[0])
        return KneePose(theta2, theta3, theta4, A, B)

    # -- 到達可能性 --------------------------------------------------------
    def is_assemblable_theta2(self, theta2: float) -> bool:
        """θ2 で三角形が閉じるか。|d − r4| ≤ r3 ≤ d + r4 (KN-8)。"""
        d = math.hypot(*self.crank_pin(theta2))
        return abs(d - self.r4) <= self.r3 <= d + self.r4

    def is_assemblable_theta4(self, theta4: float) -> bool:
        """θ4 で三角形が閉じるか。| |w| − r2 | ≤ r3 ≤ |w| + r2。"""
        B = self.rocker_pin(theta4)
        wn = math.hypot(B[0] - self.r1, B[1])
        return abs(wn - self.r2) <= self.r3 <= wn + self.r2

    # -- 伝達比・伝達角 ----------------------------------------------------
    def ratio(self, pose: KneePose) -> float:
        """伝達比 dθ4/dθ2 (KN-10)。解析式なので数値微分しない。

            dθ4/dθ2 = ((A − O2) × d) / ((B − O4) × d),   d = B − A

        分母が消えるのはカプラとロッカーが平行（死点）、分子が消えるのはカプラと
        クランクが平行になるとき。静力学は τ_θ2 = (dθ4/dθ2)·τ_θ4 で、伝達比が
        そのままトルク比になる。
        """
        d = pose.coupler
        crank = (pose.A[0] - self.r1, pose.A[1])
        den = _cross(pose.B, d)
        if abs(den) < _LEN_TOL:
            raise DeadPoint("カプラとロッカーが平行: dθ4/dθ2 が定義できない")
        return _cross(crank, d) / den

    def transmission_angle(self, pose: KneePose) -> float:
        """伝達角 γ = |θ3 − θ4| [rad] (KN-11)。カプラとロッカーのなす角。

        0 や π に近いとカプラの力がロッカーを回す成分をほとんど持たない。
        設計の目安は 40° ≤ γ ≤ 140°。
        """
        return abs(_wrap_pi(pose.theta3 - pose.theta4))

    # -- 速度・加速度 ------------------------------------------------------
    def _M(self, pose: KneePose) -> tuple[float, float, float, float, float]:
        """ループを 1 回微分した係数行列 M と、その行列式。

            M = [ −r3·sin θ3,   r4·sin θ4 ]
                [  r3·cos θ3,  −r4·cos θ4 ]

        M の行の符号と右辺の符号はセットで揃える。片方だけ −1 倍すると、速度は
        正しく出るのに加速度だけ壊れる。
        """
        s3, c3 = math.sin(pose.theta3), math.cos(pose.theta3)
        s4, c4 = math.sin(pose.theta4), math.cos(pose.theta4)
        m11, m12 = -self.r3 * s3, self.r4 * s4
        m21, m22 = self.r3 * c3, -self.r4 * c4
        det = m11 * m22 - m12 * m21          # = r3·r4·sin(θ3 − θ4)
        if abs(det) < _LEN_TOL:
            raise DeadPoint("M が特異: カプラとロッカーが平行（死点）")
        return m11, m12, m21, m22, det

    def _solve2(self, pose: KneePose, b1: float, b2: float) -> tuple[float, float]:
        m11, m12, m21, m22, det = self._M(pose)
        return ((b1 * m22 - m12 * b2) / det, (m11 * b2 - b1 * m21) / det)

    def velocity(self, pose: KneePose, omega2: float) -> tuple[float, float]:
        """(ω3, ω4)。ループ O2 + r2·e(θ2) + r3·e(θ3) − r4·e(θ4) = 0 の 1 階微分。

        右辺は _M() の M と組になっている。

            M·[ω3, ω4]ᵀ = [ r2·ω2·sin θ2, −r2·ω2·cos θ2 ]ᵀ
        """
        s2, c2 = math.sin(pose.theta2), math.cos(pose.theta2)
        return self._solve2(pose, self.r2 * omega2 * s2, -self.r2 * omega2 * c2)

    def acceleration(self, pose: KneePose, omega2: float,
                     alpha2: float) -> tuple[float, float]:
        """(α3, α4)。同じループの 2 階微分。

            M·[α3, α4]ᵀ = [ r2·α2·sinθ2 + r2·ω2²·cosθ2 + r3·ω3²·cosθ3 − r4·ω4²·cosθ4,
                           −r2·α2·cosθ2 + r2·ω2²·sinθ2 + r3·ω3²·sinθ3 − r4·ω4²·sinθ4 ]ᵀ

        右辺は velocity() と同じ M に対する組なので、符号を片方だけ触らないこと。
        """
        omega3, omega4 = self.velocity(pose, omega2)
        s2, c2 = math.sin(pose.theta2), math.cos(pose.theta2)
        s3, c3 = math.sin(pose.theta3), math.cos(pose.theta3)
        s4, c4 = math.sin(pose.theta4), math.cos(pose.theta4)
        b1 = (self.r2 * alpha2 * s2 + self.r2 * omega2 * omega2 * c2
              + self.r3 * omega3 * omega3 * c3 - self.r4 * omega4 * omega4 * c4)
        b2 = (-self.r2 * alpha2 * c2 + self.r2 * omega2 * omega2 * s2
              + self.r3 * omega3 * omega3 * s3 - self.r4 * omega4 * omega4 * s4)
        return self._solve2(pose, b1, b2)

    # -- 関節角・サーボ指令との換算（§4 の繋ぎこみ）------------------------
    # ここの 4 つ（theta4_zero, sigma_joint, phi0, sigma_motor）はすべて暫定値。
    # 値は knee_config.py にあり、実機の原点出しで確定する。
    def rocker_from_joint(self, theta4_joint: float) -> float:
        """膝関節角（伸展 0・屈曲 +）-> ロッカー絶対角。"""
        return self.sigma_joint * theta4_joint + self.theta4_zero

    def joint_from_rocker(self, theta4_rocker: float) -> float:
        """ロッカー絶対角 -> 膝関節角。"""
        return self.sigma_joint * (theta4_rocker - self.theta4_zero)

    def motor_from_crank(self, theta2: float) -> float:
        """クランク角 -> サーボ指令 φ = φ0 + σ_m·n·θ2。"""
        return self.phi0 + self.sigma_motor * self.gear * theta2

    def crank_from_motor(self, motor: float) -> float:
        """サーボ指令 -> クランク角。"""
        return (motor - self.phi0) / (self.sigma_motor * self.gear)

    def joint_to_motor(self, theta4_joint: float) -> float:
        """指令側: 脚 IK の膝関節角 -> サーボ指令。"""
        return self.motor_from_crank(self.ik(self.rocker_from_joint(theta4_joint)).theta2)

    def motor_to_joint(self, motor: float) -> float:
        """観測側: サーボの実測角 -> 膝関節角。"""
        return self.joint_from_rocker(self.fk(self.crank_from_motor(motor)).theta4)

    # -- 度で扱う境界のラッパ ----------------------------------------------
    def fk_deg(self, theta2_deg: float) -> tuple[float, float]:
        """θ2 [deg] -> (θ3, θ4) [deg]。"""
        p = self.fk(math.radians(theta2_deg))
        return math.degrees(p.theta3), math.degrees(p.theta4)

    def ik_deg(self, theta4_deg: float) -> float:
        """θ4 [deg] -> θ2 [deg]。"""
        return math.degrees(self.ik(math.radians(theta4_deg)).theta2)

    def joint_to_motor_deg(self, theta4_joint_deg: float) -> float:
        """膝関節角 [deg] -> サーボ指令 [deg]。"""
        return math.degrees(self.joint_to_motor(math.radians(theta4_joint_deg)))

    def motor_to_joint_deg(self, motor_deg: float) -> float:
        """サーボ実測角 [deg] -> 膝関節角 [deg]。"""
        return math.degrees(self.motor_to_joint(math.radians(motor_deg)))

    # -- 検算用 ------------------------------------------------------------
    def freudenstein_residual(self, pose: KneePose) -> float:
        """Freudenstein の式 (KN-9) の残差。導出の答え合わせ用。

            K1·cos(θ2 − 0) − K2·cos(θ4 − 0) + K3 = cos(θ2 − θ4)
            K1 = r1/r4,  K2 = r1/r2,  K3 = (r1² + r2² + r4² − r3²)/(2·r2·r4)

        地節は +x 上（γ = 0）に取ってあるので基準角の項が落ちる。
        """
        k1 = self.r1 / self.r4
        k2 = self.r1 / self.r2
        k3 = ((self.r1 ** 2 + self.r2 ** 2 + self.r4 ** 2 - self.r3 ** 2)
              / (2.0 * self.r2 * self.r4))
        return (k1 * math.cos(pose.theta2) - k2 * math.cos(pose.theta4) + k3
                - math.cos(pose.theta2 - pose.theta4))

    def grashof(self) -> tuple[float, float, bool]:
        """(s + l, p + q, Grashof か)。s+l ≤ p+q なら少なくとも 1 本が全周できる。"""
        r = sorted((self.r1, self.r2, self.r3, self.r4))
        return r[0] + r[3], r[1] + r[2], (r[0] + r[3]) <= (r[1] + r[2])


def knee_fourbar(side: str = "right", **overrides) -> KneeFourBar:
    """左右脚の膝リンク。``leg_ik.leg_params`` と同じ流儀。

    4 節リンクの幾何は左右共通で、変わるのはサーボの原点・向き・ギア比だけ
    （膝サーボは左右とも ID 4。どちらのバスかが左右を決める）。
    """
    side = side.lower()
    if side not in cfg.SIDES:
        raise ValueError(f"side は {cfg.SIDES} のどれか")
    kw = dict(phi0=math.radians(cfg.PHI0_DEG[side]),
              sigma_motor=cfg.SIGMA_MOTOR[side],
              gear=cfg.GEAR[side])
    kw.update(overrides)
    return KneeFourBar(**kw)


def branches_from_pose(theta2: float, theta4: float,
                       link: KneeFourBar | None = None) -> tuple[int, int]:
    """実測した 1 姿勢 (θ2, θ4) から枝 (β, ε) を決める。

    円と円の 2 交点のどちらに組んであるかは機体を組んだ時点で決まる定数なので、
    実機で 1 姿勢だけ測ればこれで確定する。結果を knee_config.BETA / EPS に書く。
    死点（2 交点が重なる姿勢）では決まらないので、そこで測ってはいけない。
    """
    link = link or KneeFourBar()
    A = link.crank_pin(theta2)
    B = link.rocker_pin(theta4)
    d = math.hypot(A[0], A[1])
    wn = math.hypot(B[0] - link.r1, B[1])
    if d < _LEN_TOL or wn < _LEN_TOL:
        raise Unreachable("退化した姿勢では枝が決まらない")
    gap = math.hypot(B[0] - A[0], B[1] - A[1]) - link.r3
    if abs(gap) > 1e-6:
        raise Unreachable(f"この (θ2, θ4) は拘束を満たしていない: |B−A| − r3 = {gap:.6g} mm")
    d_fk = _wrap_pi(theta4 - math.atan2(A[1], A[0]))
    d_ik = _wrap_pi(theta2 - math.atan2(B[1], B[0] - link.r1))
    for name, val in (("β", d_fk), ("ε", d_ik)):
        if abs(val) < 1e-9 or abs(abs(val) - math.pi) < 1e-9:
            raise DeadPoint(f"{name} が死点で決まらない姿勢")
    return (+1 if d_fk > 0.0 else -1), (+1 if d_ik > 0.0 else -1)


# --------------------------------------------------------------------------
# 自己検算（導出 §9 / 参照値の再現）
# --------------------------------------------------------------------------
def _self_test() -> None:
    link = knee_fourbar()
    lo, hi = cfg.MOTOR_SWEEP_DEG
    print("=" * 70)
    print("膝 4 節リンク  順変換 / 逆変換  自己検算")
    print("=" * 70)
    print(f"\nr1={link.r1:g} r2={link.r2:g} r3={link.r3:g} r4={link.r4:g} "
          f"[mm]  β={link.beta:+d} ε={link.eps:+d}")
    sl, pq, is_g = link.grashof()
    print(f"Grashof: s+l = {sl:g} {'<=' if is_g else '>'} p+q = {pq:g}"
          f"  -> {'Grashof' if is_g else '非 Grashof（三揺動）。どのリンクも全周できない'}")
    k1, k2 = link.r1 / link.r4, link.r1 / link.r2
    k3 = ((link.r1 ** 2 + link.r2 ** 2 + link.r4 ** 2 - link.r3 ** 2)
          / (2.0 * link.r2 * link.r4))
    print(f"Freudenstein: K1 = r1/r4 = {k1:.4f}, K2 = r1/r2 = {k2:.4f}, K3 = {k3:.4f}")

    # 組める θ2 の区間（|d − r4| ≤ r3 ≤ d + r4 を θ2 について解く）
    lo_a, hi_a = _assemblable_range(link)
    print(f"組める θ2: {math.degrees(lo_a):.2f}° 〜 {math.degrees(hi_a):.2f}° の 1 区間")

    print(f"\n[代表点]  θ2 -> θ3, θ4  (β = {link.beta:+d})")
    print(f"  {'θ2':>6} {'θ3':>9} {'θ4':>9} {'膝角 θ4−90':>11} {'γ':>8} "
          f"{'dθ4/dθ2':>9} {'ループ残差':>11}")
    for d2 in (180.0, 200.0, 220.0, 240.0, 260.0):
        pose = link.fk(math.radians(d2))
        print(f"  {d2:6.1f} {math.degrees(pose.theta3):9.3f} "
              f"{math.degrees(pose.theta4):9.3f} "
              f"{math.degrees(pose.theta4) - 90.0:11.3f} "
              f"{math.degrees(link.transmission_angle(pose)):8.3f} "
              f"{link.ratio(pose):9.4f} {pose.loop_residual(link.r3):11.2e}")

    # θ2 のスイープ
    n = int(round((hi - lo) / 0.5)) + 1
    poses = [link.fk(math.radians(lo + 0.5 * i)) for i in range(n)]
    t4 = [math.degrees(p.theta4) for p in poses]
    res = max(p.loop_residual(link.r3) for p in poses)
    rt = max(abs(_wrap_pi(link.ik(p.theta4).theta2 - p.theta2)) for p in poses)
    print(f"\n[θ2 = {lo:g}° 〜 {hi:g}° を 0.5° 刻み ({n} 点)]")
    print(f"  θ4 の範囲        {min(t4):.3f}° 〜 {max(t4):.3f}°")
    print(f"  ループ残差       最大 {res:.2e} mm")
    print(f"  IK(FK(θ2)) 誤差  最大 {math.degrees(rt):.2e} deg")

    # 設計可動域（膝角 0〜120°）
    j_lo, j_hi = cfg.JOINT_LIMIT_DEG
    m_lo = link.ik_deg(j_lo + math.degrees(link.theta4_zero))
    m_hi = link.ik_deg(j_hi + math.degrees(link.theta4_zero))
    sub = [link.fk(math.radians(m_lo + (m_hi - m_lo) * i / 400.0)) for i in range(401)]
    rr = [link.ratio(p) for p in sub]
    gg = [math.degrees(link.transmission_angle(p)) for p in sub]
    print(f"\n[設計可動域: 膝角 {j_lo:g}° 〜 {j_hi:g}°  (θ4_zero = "
          f"{math.degrees(link.theta4_zero):g}°, 暫定)]")
    print(f"  必要な θ2        {m_lo:.2f}° 〜 {m_hi:.2f}°  (振り幅 {m_hi - m_lo:.2f}°)")
    print(f"  伝達比 dθ4/dθ2   {min(rr):.3f} 〜 {max(rr):.3f}"
          f"  (区間内で {100.0 * (max(rr) / min(rr) - 1.0):.0f}% 変わる)")
    print(f"  伝達角 γ         {min(gg):.2f}° 〜 {max(gg):.2f}°")
    print("  モータ 1° が膝 2° 前後を動かす増速。トルクは約半分になるので、"
          "サーボ選定と飽和判定でこの 2 倍を効かせること。")

    fr = max(abs(link.freudenstein_residual(p)) for p in poses)
    print(f"\n[Freudenstein (KN-9) の残差]  最大 {fr:.2e}")
    print("\n" + "=" * 70)
    print("参照値と一致")
    print("=" * 70)


def _assemblable_range(link: KneeFourBar) -> tuple[float, float]:
    """組める θ2 の区間を閉形式で出す。d² = r1² + r2² + 2·r1·r2·cos θ2 を使う。"""
    out = []
    for lim in (link.r3 + link.r4, abs(link.r3 - link.r4)):
        c = ((lim * lim - link.r1 * link.r1 - link.r2 * link.r2)
             / (2.0 * link.r1 * link.r2))
        if abs(c) <= 1.0:
            out.append(math.acos(c))
    if not out:
        return 0.0, 2.0 * math.pi
    a = min(out)
    return a, 2.0 * math.pi - a


if __name__ == "__main__":
    _self_test()
