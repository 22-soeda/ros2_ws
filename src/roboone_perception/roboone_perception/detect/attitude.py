# -*- coding: utf-8 -*-
"""鉛直 u の推定。ジャイロで運び、リング面の法線で毎フレーム引き戻す。

docs/opponent_detection.pdf §5。この構成を採る理由は実測にある。

* **加速度の生値は使えない。** リング面法線を基準にした傾き誤差は、静止から
  手持ちで p95 6.40°、歩きながらで p95 28.2°、最大 46.7°。歩行時の p95 は
  水平 1 m 先で 535 mm の高さ誤差にあたり、床除去の許容 ±30 mm の 17.8 倍になる。
  加速度計は重力と機体の並進加速度の和を測っているので、これは仕様どおりの挙動。
* **ノルムゲートは効かない。** |‖a‖-g| < 0.5 を満たすフレームに限っても傾き誤差は
  p95 22.2° 残り、ノルムのずれと方向の誤差の相関は r = +0.58 しかない (§5.3)。
  重力に直交する a⊥ が乗ると方向は arctan(a⊥/g) ずれるのに、ノルムは
  sqrt(g²+a⊥²) にしか増えないので、ノルムは方向の誤差の見張りとして原理的に鈍い。
* **ジャイロの 1 フレーム予測は面法線と 0.25° (静止) / 3.3° (歩行) しか違わない。**
  ジャイロと depth は別のセンサなので、この一致は循環しない裏取りになる。

よって、絶対の傾きは毎フレーム depth (リング面の法線) から取り直し、IMU は
フレーム間をつなぐためだけに使う。加速度は初期化と、長時間リングが見えないときの
保険にだけ使う。imu_filter_madgwick の出力には依存しない (§11)。
"""

import math

import numpy as np


def _normalize(v):
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


class AttitudeEstimator:
    """鉛直 u (カメラ座標) を持ち、predict / correct の 2 段で更新する。"""

    def __init__(self, u0, blend=0.3, resid_max=0.010, angle_max_deg=12.0,
                 stale_frames=10):
        self.u = _normalize(u0)
        self.blend = float(blend)
        self.resid_max = float(resid_max)
        self.angle_max = math.radians(angle_max_deg)
        self.stale_frames = int(stale_frames)
        #: 補正が入らないまま進んだフレーム数。§9.2 の 2 番目の縮退の判定に使う
        self.since_correction = 0
        #: 直近の補正で見た法線とのなす角 [rad]。門を通らなかった理由の記録
        self.last_angle = 0.0
        self.last_resid = 0.0
        self.initialized_from_accel = False

    # ------------------------------------------------------------ 初期化
    def init_from_accel(self, accel):
        """静止しているうちに加速度で u を置く。

        IMU が静止していると加速度計は反力、すなわち上向きに約 +9.8 を読む
        (realsense_bringup の README のとおり、正立・水平では光学フレームの
        Y(下) に -9.8)。したがって加速度ベクトルの向きがそのまま上向きになる。
        起動直後の 1 回だけの用途で、以降この値は使わない。
        """
        a = np.asarray(accel, dtype=np.float64)
        if np.linalg.norm(a) < 1e-3:
            return False
        self.u = _normalize(a)
        self.initialized_from_accel = True
        return True

    # ------------------------------------------------------------ 予測
    def predict(self, omega, dt):
        """ジャイロで 1 ステップ運ぶ。式 (7)。

        慣性に固定されたベクトルのセンサ座標表現は、センサの回転と逆に回る。
        よって回転ベクトルは φ = -ω dt になる。
        """
        if dt <= 0.0:
            return self.u
        phi = -np.asarray(omega, dtype=np.float64) * float(dt)
        theta = float(np.linalg.norm(phi))
        if theta < 1e-9:
            return self.u
        k = phi / theta
        u = self.u
        self.u = _normalize(u * math.cos(theta) + np.cross(k, u) * math.sin(theta)
                            + k * float(np.dot(k, u)) * (1.0 - math.cos(theta)))
        return self.u

    def predict_samples(self, samples):
        """[(omega, dt), ...] をまとめて積分する。IMU 200Hz / depth 30Hz 用。"""
        for omega, dt in samples:
            self.predict(omega, dt)
        return self.u

    # ------------------------------------------------------------ 補正
    def correct(self, normal, resid):
        """リング面の法線で引き戻す。式 (8)。門を通ったかを返す。

        門は「残差が小さいこと」と「予測からの食い違いが小さいこと」の 2 つ。
        通らないフレームでは u を予測のまま進め、連続回数を数える。ここを
        素通しにすると、什器を斜めに貫く平面 (§5.4 の罠) に姿勢ごと引きずられる。
        """
        n = _normalize(normal)
        if float(np.dot(n, self.u)) < 0.0:      # 法線の符号は上向きに揃える
            n = -n
        self.last_resid = float(resid)
        self.last_angle = math.acos(max(-1.0, min(1.0, float(np.dot(n, self.u)))))
        if not (resid < self.resid_max and self.last_angle < self.angle_max):
            self.since_correction += 1
            return False
        b = self.blend
        self.u = _normalize((1.0 - b) * self.u + b * n)
        self.since_correction = 0
        return True

    def missed(self):
        """このフレームは面あてはめ自体ができなかった、と記録する。"""
        self.since_correction += 1

    @property
    def stale(self):
        """姿勢がジャイロ任せで劣化しているか (§9.2 の 2 番目)。"""
        return self.since_correction > self.stale_frames
