#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""片脚 6 自由度の順運動学 (FK) と逆運動学 (IK)。

``docs/脚IK導出.tex`` の解析解をそのまま実装したもの。
コード中の (FK-n) / (IK-n) は同文書の式番号を指す。

座標系（文書 §2 の定義）
    x : ロール軸 (J2/J4/J6) の方向。膝が x まわりに曲がるので x は機体の左右。
    y : ピッチ軸 (J1/J5) の方向。脚の屈曲面が y-z なので y は機体の前後。
    z : 上向き。
    ゼロ姿勢で全ての Σ_k は Σ_0 と同じ向き、脚は真下、足裏は水平。
    CAD 座標をそのまま使う場合は x <-> y を読み替える（文書 §1.1）。

関節
    θ1 股ピッチ Ry / θ2 股ロール Rx / θ3 股ヨー Rz  … 3 軸は股中心 o3 で交わる (A1)
    θ4 膝ロール Rx  θ5 足首ピッチ Ry  θ6 足首ロール Rx
    出力は「関節角」であってサーボ指令角ではない（文書 §7）。
    4 節リンク・パラレルリンクの変換 f4, f56 は別レイヤ。

寸法はすべて仮置き。値は ``leg_config.py`` にまとめてあるので、
CAD 確定後はそちらだけを書き換える（この本体は触らなくてよい）。
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

try:
    import leg_config as cfg
except ModuleNotFoundError:          # 別ディレクトリから import された場合
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import leg_config as cfg

__all__ = [
    "Rx", "Ry", "Rz",
    "LegParams", "leg_params", "Unreachable",
    "fk", "fk_pose", "joint_origins", "ik",
    "JOINT_NAMES", "JOINT_LIMITS", "cfg",
    "to_internal", "to_external",
]

JOINT_NAMES = ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")


# --------------------------------------------------------------------------
# 回転行列 (式 (1))
# --------------------------------------------------------------------------
def Rx(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, c, -s],
                     [0.0, s, c]])


def Ry(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s],
                     [0.0, 1.0, 0.0],
                     [-s, 0.0, c]])


def Rz(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0],
                     [s, c, 0.0],
                     [0.0, 0.0, 1.0]])


# --------------------------------------------------------------------------
# パラメータ
# --------------------------------------------------------------------------
# 既定値は leg_config.py から取る。差し替えるときはあちらを編集する。


def _flip_to_sign(flip) -> np.ndarray:
    """回転方向の指定 (0/1) を ±1 の配列に直す。

    flip は JOINT_NAMES をキーに持つ dict か、同じ並びの長さ 6 の列。
    0 -> +1（文書どおり）、1 -> -1（逆）。
    """
    if isinstance(flip, dict):
        missing = set(JOINT_NAMES) - set(flip)
        if missing:
            raise ValueError(f"回転方向の指定に足りない関節がある: {sorted(missing)}")
        values = [flip[k] for k in JOINT_NAMES]
    else:
        values = list(flip)
        if len(values) != 6:
            raise ValueError("回転方向の指定は長さ 6 でなければならない")
    if any(v not in (0, 1) for v in values):
        raise ValueError(f"回転方向は 0 か 1 で指定する: {values}")
    return np.array([1.0 - 2.0 * v for v in values])


@dataclass
class LegParams:
    """片脚の定数。長さの単位は mm、角度は rad。"""

    l3: float = cfg.L3
    l4: float = cfg.L4
    l5: float = cfg.L5
    l6: float = cfg.L6
    a3: float = cfg.A3_X     #: p3 の x 成分
    a4: float = cfg.A4_X     #: p4 の x 成分
    b: float = cfg.B_THIGH   #: p3 の y 成分
    #: ボディ原点 -> 股中心 o3 (Σ_0, 定数)
    p0: np.ndarray = field(
        default_factory=lambda: np.array([cfg.HIP_X, cfg.HIP_Y, cfg.HIP_Z]))
    #: o6 -> 足先基準点 (Σ_6, 定数)。None なら (0, 0, -l6)
    p6: np.ndarray | None = cfg.P6
    #: 膝の曲がる向き σ = ±1
    sigma: int = cfg.SIGMA
    #: 各軸の回転方向。0 = 文書どおり / 1 = 逆。dict でも長さ 6 の列でもよい
    flip: dict | tuple | list | np.ndarray = field(
        default_factory=lambda: dict(cfg.AXIS_FLIP))
    name: str = ""

    # __post_init__ で決まる派生量
    p3: np.ndarray = field(init=False)
    p4: np.ndarray = field(init=False)
    p5: np.ndarray = field(init=False)
    a: float = field(init=False)     # a3 + a4。x はこの和だけが効く（導出 [1]）
    l3e: float = field(init=False)   # 有効大腿長 ℓ3' = hypot(ℓ3, b)
    phi: float = field(init=False)   # 膝角オフセット φ = atan2(b, ℓ3)
    sign: np.ndarray = field(init=False)   # flip を ±1 にしたもの (6, )

    def __post_init__(self) -> None:
        if not (self.l3 > 0.0 and self.l4 > 0.0):
            raise ValueError("ℓ3, ℓ4 は正でなければならない")
        if not (0.0 <= self.l5 < self.l4):
            # ℓ4 > ℓ5 は (IK-7') の分母と符号判定で使う（文書 §5.6 末尾）
            raise ValueError("0 <= ℓ5 < ℓ4 でなければならない")
        if self.sigma not in (+1, -1):
            raise ValueError("σ は ±1")
        self.p0 = np.asarray(self.p0, dtype=float).reshape(3)
        self.p3 = np.array([self.a3, self.b, -self.l3])
        self.p4 = np.array([self.a4, 0.0, -self.l4])
        self.p5 = np.array([0.0, 0.0, -self.l5])
        if self.p6 is None:
            self.p6 = np.array([0.0, 0.0, -self.l6])
        else:
            self.p6 = np.asarray(self.p6, dtype=float).reshape(3)
        # 大腿 y オフセットの吸収（文書 §6）: ℓ3 -> ℓ3', θ4 -> θ4' = θ4 - φ
        self.a = self.a3 + self.a4
        self.l3e = math.hypot(self.l3, self.b)
        self.phi = math.atan2(self.b, self.l3)
        self.sign = _flip_to_sign(self.flip)


def leg_params(side: str = "right", **overrides) -> LegParams:
    """左右脚の既定パラメータ。side は 'right' / 'left'。"""
    side = side.lower()
    if side not in ("right", "left"):
        raise ValueError("side は 'right' か 'left'")
    lat = +1.0 if side == "right" else -1.0
    flip = cfg.AXIS_FLIP
    if side == "left" and cfg.AXIS_FLIP_LEFT is not None:
        flip = cfg.AXIS_FLIP_LEFT
    kw = dict(p0=np.array([lat * cfg.HIP_X, cfg.HIP_Y, cfg.HIP_Z]),
              flip=dict(flip) if isinstance(flip, dict) else tuple(flip),
              name=side)
    kw.update(overrides)
    return LegParams(**kw)


#: 可動域 [rad]。値そのものは leg_config.JOINT_LIMITS_DEG（deg）にある。
JOINT_LIMITS = {
    k: (math.radians(lo), math.radians(hi))
    for k, (lo, hi) in cfg.JOINT_LIMITS_DEG.items()
}
assert tuple(JOINT_LIMITS) == JOINT_NAMES, "leg_config.JOINT_LIMITS_DEG のキーと順序が違う"


class Unreachable(ValueError):
    """目標が脚の到達範囲の外（文書 §5.6「到達不能の判定」）。"""


# --------------------------------------------------------------------------
# 外部の関節角 <-> 文書の関節角
# --------------------------------------------------------------------------
# 符号は ±1 なので、どちら向きの変換も同じ掛け算で済む。
# 幾何は文書の符号（内部表現）で解き、境界でだけ掛け直す。
def to_internal(theta, prm: LegParams | None = None) -> np.ndarray:
    """外部の関節角 -> 文書の符号での関節角。"""
    prm = prm or LegParams()
    return np.asarray(theta, dtype=float).reshape(6) * prm.sign


def to_external(theta, prm: LegParams | None = None) -> np.ndarray:
    """文書の符号での関節角 -> 外部の関節角。"""
    prm = prm or LegParams()
    return np.asarray(theta, dtype=float).reshape(6) * prm.sign


# --------------------------------------------------------------------------
# 順運動学
# --------------------------------------------------------------------------
def fk(theta, prm: LegParams | None = None):
    """関節角 -> (足先位置 p, 足姿勢 R)。式 (FK-1) をそのまま評価する。

    theta : 長さ 6 の配列 (θ1..θ6) [rad]。leg_config.AXIS_FLIP を適用した符号。
    戻り値 : (p, R) = (3, ), (3, 3)  いずれも Σ_0
    """
    prm = prm or LegParams()
    t1, t2, t3, t4, t5, t6 = to_internal(theta, prm)      # 回転方向の指定を適用

    R123 = Ry(t1) @ Rx(t2) @ Rz(t3)                       # (FK-3)
    R4, R5, R6 = Rx(t4), Ry(t5), Rx(t6)
    R456 = R4 @ R5 @ R6                                    # (FK-5)

    # q: 股中心 o3 -> 足先 を Σ_3 で見たベクトル (FK-2)
    q = prm.p3 + R4 @ (prm.p4 + R5 @ (prm.p5 + R6 @ prm.p6))

    return prm.p0 + R123 @ q, R123 @ R456                  # (FK-4)


def fk_pose(theta, prm: LegParams | None = None) -> np.ndarray:
    """FK の結果を 4x4 同次変換で返す。"""
    p, R = fk(theta, prm)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def joint_origins(theta, prm: LegParams | None = None) -> np.ndarray:
    """各関節の回転中心 [o3, o4, o5, o6, 足先] を Σ_0 で返す (5, 3)。描画・検証用。"""
    prm = prm or LegParams()
    t1, t2, t3, t4, t5, t6 = to_internal(theta, prm)
    R123 = Ry(t1) @ Rx(t2) @ Rz(t3)
    R1234 = R123 @ Rx(t4)
    R12345 = R1234 @ Ry(t5)
    R = R12345 @ Rx(t6)

    o3 = prm.p0
    o4 = o3 + R123 @ prm.p3
    o5 = o4 + R1234 @ prm.p4
    o6 = o5 + R12345 @ prm.p5
    tip = o6 + R @ prm.p6
    return np.array([o3, o4, o5, o6, tip])


# --------------------------------------------------------------------------
# 逆運動学（文書 §5.5「手順のまとめ」）
# --------------------------------------------------------------------------
def ik(p, R, prm: LegParams | None = None, *, clamp: bool = True) -> np.ndarray:
    """(足先位置 p, 足姿勢 R) -> 関節角 θ1..θ6。反復なしの閉形式解。

    p     : (3, )  Σ_0 での足先基準点
    R     : (3, 3) Σ_0 での足姿勢
    clamp : True なら到達不能を最寄り姿勢に丸める。False なら Unreachable を送出。

    戻り値の符号は leg_config.AXIS_FLIP を適用したもの（fk が受け取るのと同じ）。
    式番号は roboone_kinematics/leg_kinematics.hpp の「x 成分を入れた導出」に対応。
    """
    prm = prm or LegParams()
    p = np.asarray(p, dtype=float).reshape(3)
    R = np.asarray(R, dtype=float).reshape(3, 3)

    l3, l4, l5, a = prm.l3e, prm.l4, prm.l5, prm.a

    # 1. 股中心 -> 足首ロール軸 を Σ_6 で見たベクトル (IK-1)
    r = R.T @ (p - prm.p0) - prm.p6
    rx, ry, rz = float(r[0]), float(r[1]), float(r[2])

    # 2. (X-6)
    K = float(r @ r) - a * a - l3 * l3 + l4 * l4 - l5 * l5

    # 3. θ5 についての線形式 (X-7):  P·s5 + Q·c5 = C
    P = K
    Q = 2.0 * (a * l4 - l5 * rx)
    C = 2.0 * (l4 * rx - a * l5)
    Rm = math.hypot(P, Q)
    if Rm < 1e-12:
        raise Unreachable("(X-7) が退化している (P = Q = 0)")
    ratio = C / Rm
    exact = abs(ratio) <= 1.0
    if not exact and not clamp:
        raise Unreachable(f"|C|/|(P,Q)| = {abs(ratio):.6g} > 1: x 方向に遠すぎる")
    beta = math.asin(max(-1.0, min(1.0, ratio)))
    psi = math.atan2(Q, P)

    # 4. 2 つの根から枝を選ぶ。A > 0 のうち膝が範囲内、次に cosθ5 が大きい方
    best = None
    for t5 in (beta - psi, math.pi - beta - psi):
        c5, s5 = math.cos(t5), math.sin(t5)
        # (X-8)。分母 2(ℓ4 + ℓ5·c5) は ℓ4 > ℓ5 なので 0 にならない
        A = (K + 2.0 * l5 * a * s5) / (2.0 * (l4 + l5 * c5))
        if A <= 0.0:
            continue                      # 文書 §5.6「A > 0 の仮定」
        c4 = (A - l4) / l3
        over = abs(c4) > 1.0
        if best is None or (best[0] and not over) or (over == best[0] and c5 > best[1]):
            best = (over, c5, s5, t5, A, max(-1.0, min(1.0, c4)))
    if best is None:
        raise Unreachable("A > 0 の枝が無い: 足先が股中心に近すぎる")
    over, c5, s5, t5, A, c4 = best
    if over and not clamp:
        raise Unreachable(f"cosθ4 が範囲外: 遠すぎるか近すぎる")

    # 5. 膝 (IK-9)
    t4e = prm.sigma * math.acos(c4)
    B = l3 * math.sin(t4e)
    t4 = t4e + prm.phi                    # 本来の θ4（文書 §6）

    # 6. 足首ロール (X-9)
    V = a * s5 - A * c5 - l5
    t6 = math.atan2(V * ry + B * rz, V * rz - B * ry)

    # 7. 残った回転から股 3 軸 (IK-11)〜(IK-14)
    M = R @ Rx(-t6) @ Ry(-t5) @ Rx(-t4)
    t2 = math.atan2(-M[1, 2], math.hypot(M[1, 0], M[1, 1]))
    t3 = math.atan2(M[1, 0], M[1, 1])
    t1 = math.atan2(M[0, 2], M[2, 2])

    return to_external(np.array([t1, t2, t3, t4, t5, t6]), prm)


# --------------------------------------------------------------------------
# 自己検算（文書 §8 の数値検算の再現）
# --------------------------------------------------------------------------
def _random_thetas(n: int, prm: LegParams, rng: np.random.Generator) -> np.ndarray:
    """可動域内の一様乱数姿勢。膝は σ の向きに 0〜150 度。

    JOINT_LIMITS は文書の符号での範囲なので、内部符号で作ってから
    AXIS_FLIP を適用した外部符号に直して返す（fk/ik が受け取る形）。
    """
    lo = np.array([JOINT_LIMITS[k][0] for k in JOINT_NAMES])
    hi = np.array([JOINT_LIMITS[k][1] for k in JOINT_NAMES])
    th = rng.uniform(lo, hi, size=(n, 6))
    th[:, 3] *= prm.sigma                       # 膝は分岐に合わせて符号を付ける
    th[:, 3] += prm.phi                         # 大腿オフセットぶんずらす
    return th * prm.sign                        # 内部 -> 外部


def _check_zero_pose(prm: LegParams) -> None:
    p, R = fk(np.zeros(6), prm)
    # ゼロ姿勢は全ての回転が I なので、リンクベクトルの単純な和になる
    want_p = prm.p0 + prm.p3 + prm.p4 + prm.p5 + prm.p6
    err_p = float(np.max(np.abs(p - want_p)))
    err_R = float(np.max(np.abs(R - np.eye(3))))
    print(f"  ゼロ姿勢: p = {np.array2string(p, precision=3)}"
          f"  (期待 p0 + Σp_k = {np.array2string(want_p, precision=3)})")
    print(f"           位置誤差 {err_p:.2e} mm / 姿勢誤差 {err_R:.2e}")
    assert err_p < 1e-12 and err_R < 1e-12, "ゼロ姿勢が一致しない"


def _check_roundtrip(prm: LegParams, n: int, seed: int) -> None:
    rng = np.random.default_rng(seed)
    thetas = _random_thetas(n, prm, rng)
    max_dp = max_dR = max_dth = 0.0
    n_unreachable = 0
    for th in thetas:
        p, R = fk(th, prm)
        try:
            th2 = ik(p, R, prm, clamp=False)
        except Unreachable:
            n_unreachable += 1
            continue
        p2, R2 = fk(th2, prm)
        max_dp = max(max_dp, float(np.max(np.abs(p2 - p))))
        max_dR = max(max_dR, float(np.max(np.abs(R2 - R))))
        dth = np.abs(np.arctan2(np.sin(th2 - th), np.cos(th2 - th)))
        max_dth = max(max_dth, float(np.max(dth)))
    print(f"  FK(IK(FK(θ))) 比較 {n} 姿勢:")
    print(f"           位置の最大誤差   {max_dp:.2e} mm")
    print(f"           姿勢行列の最大誤差 {max_dR:.2e}")
    print(f"           関節角の最大誤差 {math.degrees(max_dth):.2e} deg")
    print(f"           到達不能判定     {n_unreachable} 件")
    assert max_dp < 1e-9, "位置が戻らない"
    assert max_dR < 1e-12, "姿勢が戻らない"
    assert max_dth < 1e-9, "関節角が戻らない"
    assert n_unreachable == 0, "可動域内で到達不能が出た"


def _check_cosine_law(n: int, seed: int) -> None:
    """a = 0, ℓ5 = 0 で IK の cosθ4 が余弦定理に一致すること（文書 §5.7）。"""
    prm = leg_params("right", a3=0.0, a4=0.0, l5=0.0, name="a=0, ℓ5=0")
    rng = np.random.default_rng(seed)
    worst = 0.0
    for th in _random_thetas(n, prm, rng):
        p, R = fk(th, prm)
        r = R.T @ (p - prm.p0) - prm.p6
        c4_law = ((float(r @ r) - prm.l3e ** 2 - prm.l4 ** 2)
                  / (2.0 * prm.l3e * prm.l4))
        c4_ik = math.cos(to_internal(ik(p, R, prm), prm)[3] - prm.phi)
        worst = max(worst, abs(c4_law - c4_ik))
    print(f"  a=0, ℓ5=0 の cosθ4 と余弦定理の差 ({n} 姿勢): 最大 {worst:.2e}")
    assert worst < 1e-12, "余弦定理と一致しない"


def _check_closed_forms(prm: LegParams, n: int, seed: int) -> None:
    """導出の成分式 (FK-3)(FK-5)(X-1)-(X-6)(X-8) を FK の行列積と照合する。

    FK は式 (FK-1) の行列積で実装してあるので、展開結果の成分式と
    突き合わせれば導出の代数を検算したことになる。
    """
    rng = np.random.default_rng(seed)
    worst = {"FK-3 R123": 0.0, "FK-5 R456": 0.0, "X-1..5 r": 0.0,
             "X-6,8 A": 0.0, "4x4 chain": 0.0}
    l3, l4, l5, a = prm.l3e, prm.l4, prm.l5, prm.a

    for th_ext in _random_thetas(n, prm, rng):
        # 成分式は内部符号（AXIS_FLIP 適用前）で書かれている
        t1, t2, t3, t4, t5, t6 = to_internal(th_ext, prm)
        c1, s1 = math.cos(t1), math.sin(t1)
        c2, s2 = math.cos(t2), math.sin(t2)
        c3, s3 = math.cos(t3), math.sin(t3)
        c5, s5 = math.cos(t5), math.sin(t5)
        c6, s6 = math.cos(t6), math.sin(t6)
        c4, s4 = math.cos(t4), math.sin(t4)

        # (FK-3)
        R123_doc = np.array([
            [c1 * c3 + s1 * s2 * s3, s1 * s2 * c3 - c1 * s3, s1 * c2],
            [c2 * s3, c2 * c3, -s2],
            [-s1 * c3 + c1 * s2 * s3, s1 * s3 + c1 * s2 * c3, c1 * c2],
        ])
        worst["FK-3 R123"] = max(worst["FK-3 R123"],
                                 float(np.max(np.abs(R123_doc - Ry(t1) @ Rx(t2) @ Rz(t3)))))

        # (FK-5)
        R456_doc = np.array([
            [c5, s5 * s6, s5 * c6],
            [s4 * s5, c4 * c6 - s4 * c5 * s6, -c4 * s6 - s4 * c5 * c6],
            [-c4 * s5, s4 * c6 + c4 * c5 * s6, -s4 * s6 + c4 * c5 * c6],
        ])
        worst["FK-5 R456"] = max(worst["FK-5 R456"],
                                 float(np.max(np.abs(R456_doc - Rx(t4) @ Ry(t5) @ Rx(t6)))))

        p, R = fk(th_ext, prm)
        r_impl = R.T @ (p - prm.p0) - prm.p6

        # (X-1)〜(X-5): r の成分式。p6 には依らない（r は o3 -> o6）
        t4e = t4 - prm.phi
        A = l3 * math.cos(t4e) + l4
        B = l3 * math.sin(t4e)
        V = a * s5 - A * c5 - l5
        r_doc = np.array([a * c5 + A * s5, -B * c6 + V * s6, B * s6 + V * c6])
        worst["X-1..5 r"] = max(worst["X-1..5 r"], float(np.max(np.abs(r_doc - r_impl))))

        # (X-6)(X-8): K から A が戻ること
        K = float(r_impl @ r_impl) - a * a - l3 * l3 + l4 * l4 - l5 * l5
        A_back = (K + 2.0 * l5 * a * s5) / (2.0 * (l4 + l5 * c5))
        worst["X-6,8 A"] = max(worst["X-6,8 A"], abs(A_back - A))

        # 4x4 同次変換の素直な連鎖（FK の独立実装）。
        # リンクベクトルは次の関節の回転と組になる。
        def T(rot, trans):
            m = np.eye(4)
            m[:3, :3] = rot
            m[:3, 3] = trans
            return m
        chain = (T(np.eye(3), prm.p0)
                 @ T(Ry(t1), np.zeros(3)) @ T(Rx(t2), np.zeros(3)) @ T(Rz(t3), np.zeros(3))
                 @ T(Rx(t4), prm.p3) @ T(Ry(t5), prm.p4) @ T(Rx(t6), prm.p5)
                 @ T(np.eye(3), prm.p6))
        worst["4x4 chain"] = max(worst["4x4 chain"],
                                 max(float(np.max(np.abs(chain[:3, 3] - p))),
                                     float(np.max(np.abs(chain[:3, :3] - R)))))

    for k, v in worst.items():
        print(f"  {k:<12s} 最大差 {v:.2e}")
        assert v < 1e-10, f"{k} が一致しない"


def _check_x_split(n: int, seed: int) -> None:
    """a3 と a4 は和だけが効く（導出 [1]）。"""
    splits = [(11.0, -4.0), (-20.0, 27.0), (7.0, 0.0), (0.0, 7.0)]
    base = leg_params("right", a3=splits[0][0], a4=splits[0][1], b=5.0)
    rng = np.random.default_rng(seed)
    wp = wR = wth = 0.0
    for th in _random_thetas(n, base, rng):
        p0_, R0_ = fk(th, base)
        for a3, a4 in splits:
            prm = leg_params("right", a3=a3, a4=a4, b=5.0)
            p, R = fk(th, prm)
            wp = max(wp, float(np.max(np.abs(p - p0_))))
            wR = max(wR, float(np.max(np.abs(R - R0_))))
            th2 = ik(p, R, prm, clamp=False)
            d = np.abs(np.arctan2(np.sin(th2 - th), np.cos(th2 - th)))
            wth = max(wth, float(np.max(d)))
    print(f"  a3+a4 = {sum(splits[0]):g} を保ったまま 4 通りに分け直し x {n} 姿勢:")
    print(f"           姿勢の差 位置 {wp:.2e} mm / 姿勢 {wR:.2e}  (0 であるべき)")
    print(f"           往復の関節角最大誤差 {math.degrees(wth):.2e} deg")
    assert wp < 1e-12 and wR < 1e-14, "x の分け方で姿勢が変わってしまう"
    assert wth < 1e-9, "分け方を変えると往復しない"


def _check_axis_flip(n: int, seed: int) -> None:
    """AXIS_FLIP が符号だけを変えて幾何は変えないこと、全 64 通りで往復すること。"""
    base = leg_params("right", a3=9.0, a4=-3.0, b=4.0,
                      flip=(0, 0, 0, 0, 0, 0), name="flip=0")
    rng = np.random.default_rng(seed)
    thetas = _random_thetas(n, base, rng)      # base は sign=+1 なので内部符号そのもの

    worst_pose = worst_round = 0.0
    for mask in range(64):                     # 2^6 通りの 0/1 の組み合わせ
        flip = tuple((mask >> k) & 1 for k in range(6))
        prm = leg_params("right", a3=9.0, a4=-3.0, b=4.0, flip=flip)
        for th_int in thetas:
            th_ext = th_int * prm.sign         # 同じ姿勢を外部符号で書いたもの
            p0_, R0_ = fk(th_int, base)
            p1_, R1_ = fk(th_ext, prm)
            worst_pose = max(worst_pose,
                             float(np.max(np.abs(p1_ - p0_))),
                             float(np.max(np.abs(R1_ - R0_))))
            th2 = ik(p1_, R1_, prm, clamp=False)
            d = np.abs(np.arctan2(np.sin(th2 - th_ext), np.cos(th2 - th_ext)))
            worst_round = max(worst_round, float(np.max(d)))

    print(f"  64 通りの AXIS_FLIP x {n} 姿勢:")
    print(f"           flip による姿勢の差   {worst_pose:.2e}  (0 であるべき)")
    print(f"           往復の関節角最大誤差 {math.degrees(worst_round):.2e} deg")
    assert worst_pose < 1e-12, "flip が幾何を変えてしまっている"
    assert worst_round < 1e-9, "flip 下で往復しない"

    for bad in ({"hip_pitch": 2}, (0, 0, 0, 0, 0), (0, 0, 0, 0, 0, -1)):
        try:
            leg_params("right", flip=bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"不正な回転方向の指定が通ってしまった: {bad}")
    print("  不正な指定 (0/1 以外・長さ違い・キー不足) はすべて弾いた")


def _check_unreachable(prm: LegParams) -> None:
    """到達不能の 3 条件が送出され、clamp=True では最寄り姿勢に落ちること。"""
    reach = prm.l3e + prm.l4 + prm.l5
    # r_x^2 ~ ℓ3'^2 - ℓ4^2 + ℓ5^2 + a^2 なら K ~ 0 で (X-7) の |C| > |(P,Q)| を踏む
    rx_crit = math.sqrt(max(0.0, prm.l3e ** 2 - prm.l4 ** 2 + prm.l5 ** 2 + prm.a ** 2))
    cases = [
        ("遠すぎる", prm.p0 + np.array([0.0, 0.0, -(reach + prm.l6 + 50.0)])),
        ("近すぎる", prm.p0 + np.array([0.0, 0.0, -(prm.l6 + 1.0)])),
        ("x 方向に遠い", prm.p0 + prm.p6 + np.array([rx_crit, 0.0, 0.0])),
    ]
    n_raised = 0
    for label, target in cases:
        try:
            ik(target, np.eye(3), prm, clamp=False)
        except Unreachable as exc:
            n_raised += 1
            print(f"  {label}: {exc}")
        else:
            print(f"  {label}: 送出されなかった")
    assert n_raised == len(cases), "到達不能が検出されない条件がある"


def _self_test(n: int = 20000, seed: int = 0) -> None:
    print("=" * 70)
    print("片脚 FK / IK 自己検算  (Python 参照実装 / x 成分の拡張込み)")
    print("=" * 70)
    for side in ("right", "left"):
        prm = leg_params(side)
        print(f"\n[{side}] ℓ3={prm.l3:g} ℓ4={prm.l4:g} ℓ5={prm.l5:g} ℓ6={prm.l6:g} "
              f"a3={prm.a3:g} a4={prm.a4:g} (a={prm.a:g}) b={prm.b:g} σ={prm.sigma:+d}")
        print(f"       p0={np.array2string(prm.p0, precision=1)} "
              f"flip={tuple(int(v) for v in (1 - prm.sign) // 2)}")
        _check_zero_pose(prm)
        _check_roundtrip(prm, n, seed)

    print("\n[x 成分あり  a3=11, a4=-4, b=7]")
    prm_x = leg_params("right", a3=11.0, a4=-4.0, b=7.0)
    _check_zero_pose(prm_x)
    _check_roundtrip(prm_x, n, seed + 1)

    print("\n[導出の閉形式との照合]")
    _check_closed_forms(prm_x, min(n, 2000), seed + 2)

    print("\n[x の分け方の不変性 (導出 [1])]")
    _check_x_split(min(n, 500), seed + 3)

    print("\n[a = 0, ℓ5 = 0 での検算]")
    _check_cosine_law(min(n, 2000), seed + 4)

    print("\n[膝の分岐 σ = -1 (x 成分あり)]")
    _check_roundtrip(leg_params("right", a3=-6.0, a4=2.5, b=-3.0, sigma=-1),
                     min(n, 5000), seed + 5)

    print("\n[軸の回転方向 AXIS_FLIP]")
    _check_axis_flip(min(n, 60), seed + 6)

    print("\n[到達不能の判定]")
    _check_unreachable(leg_params("right", a3=11.0, a4=-4.0))

    print("\n" + "=" * 70)
    print("すべて一致")
    print("=" * 70)


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="片脚 FK/IK の自己検算 (Python 参照実装)")
    ap.add_argument("-n", type=int, default=20000, help="乱数姿勢の数 (既定 20000)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    _self_test(args.n, args.seed)
