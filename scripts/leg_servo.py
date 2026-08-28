#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""片脚の関節角 <-> サーボ指令。脚 IK と膝 4 節リンクを繋ぐ層。

``leg_ik.py`` が出すのは「関節角」であってサーボ指令角ではない（脚IK導出 §7）。
実機との間には変換が挟まるので、ここで束ねて

    指令側: 足先目標 -> ik() -> (θ1..θ6) -> servo_from_joints() -> サーボ指令
    観測側: サーボ実測 -> joints_from_servo() -> (θ1..θ6) -> fk() -> 重心位置

の 2 行で書けるようにする。内訳は

    J1-J3（股 3 軸）  サーボ直結。変換なし
    J4（膝）          4 節リンク。``knee_fourbar.py``  (KN-5)/(KN-7)
    J5・J6（足首）    パラレルリンク。**Python 版は未実装**（C++ の
                      ``roboone_kinematics/ankle_parallel.hpp`` にある）。
                      ここでは素通しし、素通しであることを status で返す。

C++ 版は ``roboone_kinematics/leg_servo.hpp`` で、あちらは足首まで含めて 3 つとも
揃っている。突き合わせは ``crosscheck_knee.py``（膝の部分）で行う。

===========================================================================
膝の繋ぎこみ（このモジュールの主眼）
===========================================================================
脚 IK の公開角 θ4（AXIS_FLIP 適用後）と 4 節リンクのロッカー絶対角は別物で、
2 段の読み替えで結ばれる。

    [1] 公開角 -> 曲げ量 bend（伸展 0・屈曲 +）
          θ4_doc = θ4_pub · sign[knee]              AXIS_FLIP を外す
          bend   = σ_leg · (θ4_doc − φ)             解析解の θ4 = σ·bend + φ
        既定値（AXIS_FLIP なし・b = 0・σ_leg = -1）では bend = −θ4_pub になる。
        C++ 側は Σ_B なので (X-swap) がもう 1 段入り、そちらでは bend = +θ4_B。
        **同じ姿勢を指しているが公開角の符号が違う**ので、値を直接見比べない。

    [2] 曲げ量 -> ロッカー角
          θ4_rocker = σ_knee · bend + θ4_zero       knee_config.py

θ4_zero = 89.3 deg は「脚が伸び切った姿勢（T ポーズ）でのロッカー角」で確定値。
σ_knee はそこから一意に決まる（+1）。

    python3 scripts/leg_servo.py        # 繋ぎこみの自己検算
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import knee_fourbar  # noqa: E402
import leg_ik  # noqa: E402

__all__ = [
    "LegServo", "leg_servo", "AnkleNotImplemented",
    "knee_bend_from_leg_angle", "leg_angle_from_knee_bend",
]


class AnkleNotImplemented(NotImplementedError):
    """足首パラレルリンクの Python 版は無い（C++ の ankle_parallel.hpp を使う）。"""


# --------------------------------------------------------------------------
# 膝: 脚 IK の公開角 <-> 曲げ量
# --------------------------------------------------------------------------
# 掛かるのは ±1 だけなので、どちら向きの変換も同じ式でよい。
def knee_bend_from_leg_angle(theta4_pub: float, prm: leg_ik.LegParams) -> float:
    """脚 IK の公開角 θ4 -> 膝の曲げ量（伸展 0・屈曲 +）。"""
    return prm.sigma * (theta4_pub * prm.sign[leg_ik.JOINT_NAMES.index("knee")] - prm.phi)


def leg_angle_from_knee_bend(bend: float, prm: leg_ik.LegParams) -> float:
    """膝の曲げ量 -> 脚 IK の公開角 θ4。"""
    return ((prm.sigma * bend + prm.phi)
            * prm.sign[leg_ik.JOINT_NAMES.index("knee")])


# --------------------------------------------------------------------------
@dataclass
class LegServo:
    """片脚ぶんの変換層。"""

    leg: leg_ik.LegParams = field(default_factory=leg_ik.LegParams)
    knee: knee_fourbar.KneeFourBar = field(default_factory=knee_fourbar.KneeFourBar)

    # -- 膝だけの近道 ------------------------------------------------------
    def knee_servo_from_angle(self, theta4_pub: float) -> float:
        """脚 IK の公開角 θ4 -> 膝サーボ指令。到達不能なら Unreachable。"""
        return self.knee.joint_to_motor(knee_bend_from_leg_angle(theta4_pub, self.leg))

    def knee_angle_from_servo(self, servo: float) -> float:
        """膝サーボ実測角 -> 脚 IK の公開角 θ4。"""
        return leg_angle_from_knee_bend(self.knee.motor_to_joint(servo), self.leg)

    # -- 6 軸まとめて ------------------------------------------------------
    def servo_from_joints(self, theta, *, ankle: bool = False) -> np.ndarray:
        """関節角 θ1..θ6 -> サーボ指令。

        ankle=False（既定）なら J5・J6 は素通し。True にすると
        AnkleNotImplemented を送出する（呼び側が素通しに気づけるように）。
        """
        if ankle:                                # 未実装の通知は入力に依らない
            raise AnkleNotImplemented(
                "足首は C++ の roboone_kinematics/ankle_parallel.hpp を使う")
        theta = np.asarray(theta, dtype=float).reshape(6)
        out = theta.copy()                       # J1-J3 は直結
        out[3] = self.knee_servo_from_angle(float(theta[3]))
        return out

    def joints_from_servo(self, servo, *, ankle: bool = False) -> np.ndarray:
        """サーボ実測角 -> 関節角 θ1..θ6。"""
        if ankle:
            raise AnkleNotImplemented(
                "足首は C++ の roboone_kinematics/ankle_parallel.hpp を使う")
        servo = np.asarray(servo, dtype=float).reshape(6)
        out = servo.copy()
        out[3] = self.knee_angle_from_servo(float(servo[3]))
        return out


def leg_servo(side: str = "right", **overrides) -> LegServo:
    """左右脚の変換層。

    膝の平面幾何・枝・θ4_zero は左右共通（文書 §11）で、左右で違うのはサーボの
    回転方向 σ_m と、そこから決まる原点 φ0 だけ。**左右別の値を拾うために
    dataclass を直に作らず knee_fourbar(side) を通すこと**。
    """
    return LegServo(leg=leg_ik.leg_params(side),
                    knee=knee_fourbar.knee_fourbar(side, **overrides))


# --------------------------------------------------------------------------
# 自己検算
# --------------------------------------------------------------------------
def _self_test() -> None:
    D, R = math.degrees, math.radians
    ls = leg_servo("right")
    prm, link = ls.leg, ls.knee
    print("=" * 70)
    print("脚 IK <-> 膝 4 節リンク <-> サーボ指令  繋ぎこみの検算")
    print("=" * 70)
    print(f"\nθ4_zero = {D(link.theta4_zero):g}°（T ポーズ・確定値） "
          f"σ_knee = {link.sigma_joint:+d} / σ_leg = {prm.sigma:+d} "
          f"φ = {D(prm.phi):g}°")

    # 1. 曲げ量 0 で脚が伸び切ること（T ポーズ）を関節位置で確かめる
    theta = np.zeros(6)
    theta[3] = leg_angle_from_knee_bend(0.0, prm)
    o = leg_ik.joint_origins(theta, prm)
    straight = float(np.linalg.norm(o[2] - o[0]))     # o3 -> o5
    want = prm.l3e + prm.l4
    print("\n[T ポーズ] 曲げ量 0 のとき")
    print(f"  公開角 θ4      {D(theta[3]):.6f}°")
    print(f"  |o3 → o5|      {straight:.6f} mm  (伸び切り ℓ3' + ℓ4 = {want:.6f} mm)")
    print(f"  ロッカー角     {D(link.rocker_from_joint(0.0)):.4f}°")
    print(f"  クランク角 θ2  {link.ik_deg(D(link.rocker_from_joint(0.0))):.4f}°")
    assert abs(straight - want) < 1e-9, "曲げ量 0 で脚が伸び切っていない"
    assert abs(D(link.rocker_from_joint(0.0)) - 89.3) < 1e-9, "T ポーズのロッカー角が違う"

    # 2. σ_knee の一意性
    print("\n[σ_knee の一意性]")
    for sig in (+1, -1):
        alt = knee_fourbar.KneeFourBar(sigma_joint=sig)
        reached, rmin = 0.0, float("inf")
        for i in range(1501):
            bend = R(i / 10.0)
            try:
                p = alt.ik(alt.rocker_from_joint(bend))
                rmin = min(rmin, abs(alt.ratio(p)))
            except (knee_fourbar.Unreachable, knee_fourbar.DeadPoint):
                break
            reached = i / 10.0
        print(f"  σ = {sig:+d}: 屈曲 0〜{reached:.1f}° まで到達（伝達比の最小 {rmin:.3f}）")
        if sig == link.sigma_joint:
            assert reached >= 150.0, "設計可動域に届かない"
        else:
            assert reached < 150.0, "σ の一意性が崩れている"

    # 3. 公開角 -> サーボ -> 公開角 の往復
    worst = 0.0
    servos = []
    for i in range(301):
        bend = R(i * 0.5)
        th4 = leg_angle_from_knee_bend(bend, prm)
        s = ls.knee_servo_from_angle(th4)
        servos.append(s)
        d = ls.knee_angle_from_servo(s) - th4
        worst = max(worst, abs(math.atan2(math.sin(d), math.cos(d))))
    print(f"\n[往復] θ4 -> サーボ -> θ4（屈曲 0〜150°）最大誤差 {D(worst):.2e} deg")
    print(f"       必要なサーボ振り幅 {D(max(servos) - min(servos)):.2f}°"
          f"（{D(min(servos)):.2f}° 〜 {D(max(servos)):.2f}°）")
    assert worst < 1e-9, "関節角 <-> サーボ指令が往復しない"

    # 4. 脚 IK 全体との繋ぎこみ（可動域内の乱数姿勢）
    rng = np.random.default_rng(20260828)
    lo = np.array([leg_ik.JOINT_LIMITS[k][0] for k in leg_ik.JOINT_NAMES])
    hi = np.array([leg_ik.JOINT_LIMITS[k][1] for k in leg_ik.JOINT_NAMES])
    worst_all, n_ok = 0.0, 0
    for _ in range(2000):
        th = rng.uniform(lo, hi)
        th[3] = leg_angle_from_knee_bend(abs(th[3]), prm)
        th[:3] *= prm.sign[:3]
        th[4:] *= prm.sign[4:]
        p, Rm = leg_ik.fk(th, prm)
        try:
            back = leg_ik.ik(p, Rm, prm, clamp=False)
        except leg_ik.Unreachable:
            continue
        s = ls.servo_from_joints(back)
        again = ls.joints_from_servo(s)
        d = again[3] - th[3]
        worst_all = max(worst_all, abs(math.atan2(math.sin(d), math.cos(d))))
        n_ok += 1
    print(f"\n[脚 IK と接続] fk → ik → サーボ → 関節角 を {n_ok} 姿勢: "
          f"膝の最大誤差 {D(worst_all):.2e} deg")
    assert n_ok > 1500, "可動域内なのに解けない姿勢が多すぎる"
    assert worst_all < 1e-9, "脚 IK と繋ぐと往復しない"

    # 5. 設計可動域でのサーボ角と伝達比の対応表
    print("\n[対応表] 膝の曲げ量 -> ロッカー角 -> クランク角（= サーボ角）")
    print(f"  {'曲げ量':>7} {'θ4_rocker':>11} {'θ2':>9} {'dθ4/dθ2':>9} {'γ':>8}")
    for j in (0.0, 30.0, 60.0, 90.0, 120.0, 150.0):
        pose = link.ik(link.rocker_from_joint(R(j)))
        print(f"  {j:7.1f}° {D(pose.theta4):10.2f}° {D(pose.theta2):8.2f}° "
              f"{link.ratio(pose):9.3f} {D(link.transmission_angle(pose)):7.2f}°")

    print("\n" + "=" * 70)
    print("すべて一致")
    print("=" * 70)


if __name__ == "__main__":
    _self_test()
