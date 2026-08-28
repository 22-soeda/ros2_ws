# -*- coding: utf-8 -*-
"""RealSense の IMU から姿勢を EKF で推定し、3D で見るビジュアライザ。

    # 先にカメラを上げておく（別ターミナル）
    ros2 launch realsense_bringup realsense.launch.py

    # 姿勢推定サーバ（既定で ssh 元から見える IP に bind する）
    python3 src/realsense_bringup/viz/serve_imu3d.py

    # 実機なしで表示だけ確かめる
    python3 src/realsense_bringup/viz/serve_imu3d.py --demo

ブラウザで http://<起動時に表示される IP>:8104/ を開く。

===========================================================================
これは何のためのものか
===========================================================================
**ロール・ピッチ・ヨーと実際の機体の傾きの対応付けが合っているかを、目で確かめる**
ための道具。カメラを手で持って傾けたときに、画面の矢印と数字が同じように動けば
対応付けは合っている。

推定した姿勢は publish しない（読むだけ）。本番の姿勢推定は
docs/ros-architecture.md どおり imu_filter が /imu/data に出す担当で、これは
その手前の「軸の向きが合っているか」を確かめる検証用。

===========================================================================
座標系（ここが確認したい本体）
===========================================================================
/camera/imu の frame_id は ``camera_imu_optical_frame``。光学フレームなので

    X = 右 / Y = 下 / Z = 前（レンズの向き）

一方この機体の Σ_B は REP-103 で、legs3d などの表示と揃えて

    x = 前 / y = 左 / z = 上

なので、光学 -> 機体 の載せ替えは

    x_b (前) = +z_o        y_b (左) = -x_o        z_b (上) = -y_o

これが ``M_OPT2BODY``。EKF はすべて Σ_B で回す。画面には光学フレームの生値と
Σ_B に直した値の両方を出すので、どちらで見ているかが常に分かる。

ロール・ピッチ・ヨーは ROS の標準どおり Z-Y-X（内因性）で、

    roll  φ : x(前) 軸まわり。正 = 右下がり
    pitch θ : y(左) 軸まわり。正 = 機首下げ（お辞儀）
    yaw   ψ : z(上) 軸まわり。正 = 左旋回

===========================================================================
EKF（誤差状態カルマンフィルタ / MEKF）
===========================================================================
状態は クォータニオン q（世界 <- 機体）と ジャイロバイアス b。姿勢そのものを
線形カルマンに載せると正規化が崩れるので、q は非線形のまま持って、共分散だけ
「小さな姿勢誤差 δθ とバイアス誤差 δb」の 6 次元で持つ（誤差状態形式）。

  予測 : q <- q (x) Δq(ω - b, dt)、  Φ = I + F dt、  F = [[-[ω]x, -I], [0, 0]]
  更新 : 静止していれば加速度計は「上向きに +g」を測る。これを重力の向きの
         観測として使う。 h(q) = R(q)^T [0,0,g]、  H = [ [h]x , 0 ]

**ヨーは加速度計では直せない**（重力を軸まわりに回してもベクトルが変わらない）。
6 軸 IMU だけではヨーは観測不能で、ジャイロの積分としてゆっくりドリフトする。
地磁気を足すか、歩行なら足裏の接地拘束で押さえるしかない。画面でもそう表示する。

加速度が |a| ≒ g から外れているときは機体が動いている（重力以外の加速度が
乗っている）ので、その分だけ観測ノイズを膨らませて重みを下げる。
"""

from __future__ import annotations

import argparse
import json
import math
import os
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import subprocess
import threading
import time
from urllib.parse import urlparse

import numpy as np

_HERE = Path(__file__).resolve().parent
PAGE = _HERE / 'imu3d.html'

G = 9.80665

#: 光学フレーム (X右, Y下, Z前) -> Σ_B (x前, y左, z上)
M_OPT2BODY = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])


# ---------------------------------------------------------------------------
# クォータニオンまわり（w, x, y, z）
# ---------------------------------------------------------------------------
def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])


def quat_norm(q):
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    q = q / n
    return -q if q[0] < 0.0 else q


def quat_from_rotvec(v):
    """微小回転ベクトル [rad] -> クォータニオン。"""
    th = float(np.linalg.norm(v))
    if th < 1e-9:
        return quat_norm(np.array([1.0, 0.5*v[0], 0.5*v[1], 0.5*v[2]]))
    ax = v / th
    s = math.sin(th * 0.5)
    return np.array([math.cos(th*0.5), ax[0]*s, ax[1]*s, ax[2]*s])


def quat_to_R(q):
    """世界 <- 機体 の回転行列。"""
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def R_to_rpy(R):
    """Z-Y-X（ヨー・ピッチ・ロールの順に外側から）で分解 [rad]。"""
    pitch = math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2]))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
    return quat_norm(np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ]))


def tilt_from_accel(a):
    """加速度だけから出した傾き [rad]。フィルタ無しの参照値。

    静止していれば a は「上向きに +g」なので、ロールとピッチはこれだけで決まる
    （ヨーは決まらない）。EKF の出力がこれと合っているかで実装を検算できる。
    """
    roll = math.atan2(a[1], a[2])
    pitch = math.atan2(-a[0], math.hypot(a[1], a[2]))
    return roll, pitch


# ---------------------------------------------------------------------------
# EKF
# ---------------------------------------------------------------------------
class AttitudeEKF:
    """誤差状態カルマンフィルタ。すべて Σ_B（x前 / y左 / z上）で扱う。

    観測できるのはロールとピッチだけ。ヨーは ω の積分で、時間とともにドリフトする。
    """

    def __init__(self, sigma_g=0.0030, sigma_a=0.0130, sigma_bg=2.0e-4,
                 accel_gate=1.2, bias_max=0.05):
        # 既定値は rs_imu_test の実測（静止時の標準偏差 gyro 0.0029 rad/s,
        # accel 0.0126 m/s^2）から取ってある。歩行中の振動は含まないので、
        # 実機で暴れるようなら sigma_a を上げる。
        self.sigma_g = sigma_g       # ジャイロの白色ノイズ [rad/s/√Hz]
        self.sigma_a = sigma_a       # 加速度計のノイズ [m/s^2]
        self.sigma_bg = sigma_bg     # バイアスのランダムウォーク [rad/s^2/√Hz]
        self.accel_gate = accel_gate  # |a|-g がこれ [m/s^2] を超えたら重みを下げる
        # バイアス推定の上限 [rad/s]。**鉛直まわりのバイアスは観測できない**ので、
        # 押さえないと手で振り回したぶんの回転をバイアスが吸ってしまう
        # （実機で 0.32 rad/s = 18°/s まで育ち、置いた途端にヨーが流れた）。
        # rs_imu_test の実測が最大 0.005 rad/s なので、10 倍の余裕を取ってこの値。
        self.bias_max = bias_max
        self.bias_clipped = 0
        self.reset()

    def reset(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.b = np.zeros(3)
        self.P = np.diag([0.3**2]*3 + [0.02**2]*3)
        self.ready = False
        self.n = 0
        self.gate_reject = 0
        self.bias_clipped = 0
        self.clip_at = -10**9

    def init_from_accel(self, a):
        """最初の加速度からロール・ピッチを決める。ヨーは 0 から始める。"""
        roll, pitch = tilt_from_accel(a)
        self.q = rpy_to_quat(roll, pitch, 0.0)

        # 観測不能なのは「機体 z 軸まわり」ではなく **重力方向まわり** の回転。
        # H = [h]x の零空間はちょうど h（= 機体系で見た重力の向き）なので、
        # 初期共分散も機体軸ではなくこの向きを基準に置く。
        # diag([小, 小, 大]) と書くと、傾いた姿勢で起動したときに大きい値が
        # 観測可能な方向へ漏れ、最初の更新でヨーに一発ぶんの誤差が残る。
        v = a / (np.linalg.norm(a) or 1.0)
        vv = np.outer(v, v)
        # 重力方向まわり = ヨー。ここは「今を 0 と決めた」ので初期値は小さく、
        # 以後 Q（ジャイロのノイズとバイアス）ぶんだけ素直に広がっていく。
        P_th = (0.05**2) * (np.eye(3) - vv) + (0.02**2) * vv
        self.P = np.zeros((6, 6))
        self.P[0:3, 0:3] = P_th
        self.P[3:6, 3:6] = np.eye(3) * (0.02**2)
        self.ready = True

    def predict(self, w, dt):
        w_hat = w - self.b
        self.q = quat_norm(quat_mul(self.q, quat_from_rotvec(w_hat * dt)))

        F = np.zeros((6, 6))
        F[0:3, 0:3] = -skew(w_hat)
        F[0:3, 3:6] = -np.eye(3)
        Phi = np.eye(6) + F * dt
        Q = np.zeros((6, 6))
        Q[0:3, 0:3] = np.eye(3) * (self.sigma_g**2 * dt)
        Q[3:6, 3:6] = np.eye(3) * (self.sigma_bg**2 * dt)
        self.P = Phi @ self.P @ Phi.T + Q

    def update_accel(self, a):
        R = quat_to_R(self.q)
        h = R.T @ np.array([0.0, 0.0, G])     # 静止時に期待される加速度計の読み
        H = np.zeros((3, 6))
        H[0:3, 0:3] = skew(h)

        # 重力以外の加速度が乗っているぶん、観測を信用しない
        err = abs(float(np.linalg.norm(a)) - G)
        scale = 1.0 + (err / self.accel_gate)**2 * 100.0
        if err > self.accel_gate:
            self.gate_reject += 1
        Rm = np.eye(3) * (self.sigma_a**2 * scale)

        y = a - h
        S = H @ self.P @ H.T + Rm
        K = self.P @ H.T @ np.linalg.inv(S)
        d = K @ y

        self.q = quat_norm(quat_mul(self.q, quat_from_rotvec(d[0:3])))
        b = self.b + d[3:6]
        if np.any(np.abs(b) > self.bias_max):
            self.bias_clipped += 1
            self.clip_at = self.n            # 直近に当たったのがいつかを覚えておく
            b = np.clip(b, -self.bias_max, self.bias_max)
        self.b = b
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T   # Joseph 形（対称を保つ）
        self.P = 0.5 * (self.P + self.P.T)

    def step(self, w, a, dt):
        if not self.ready:
            self.init_from_accel(a)
            return
        if dt <= 0.0 or dt > 0.2:
            return                     # 抜けや巻き戻しは捨てる（積分を壊さない）
        self.predict(w, dt)
        self.update_accel(a)
        self.n += 1

    def zero_yaw(self):
        """今の向きをヨー 0 にする（ロール・ピッチは触らない）。"""
        r, p, _ = R_to_rpy(quat_to_R(self.q))
        self.q = rpy_to_quat(r, p, 0.0)


# ---------------------------------------------------------------------------
# IMU の取り込み
# ---------------------------------------------------------------------------
class ImuState:
    """EKF と最新値を抱える。ROS のコールバックと HTTP から触るのでロックする。"""

    def __init__(self, trace_len=600):
        self.lock = threading.Lock()
        self.ekf = AttitudeEKF()
        self.t_last = None
        self.t0 = None
        self.count = 0
        self.hz = 0.0
        self._hz_t = None
        self._hz_n = 0
        self.frame_id = ''
        self.a_opt = np.zeros(3)
        self.w_opt = np.zeros(3)
        self.a_body = np.zeros(3)
        self.w_body = np.zeros(3)
        self.trace = deque(maxlen=trace_len)
        self.last_wall = 0.0
        self.error = ''

    def feed(self, t, frame_id, a_opt, w_opt):
        with self.lock:
            a_b = M_OPT2BODY @ a_opt
            w_b = M_OPT2BODY @ w_opt
            dt = 0.0 if self.t_last is None else (t - self.t_last)
            self.t_last = t
            if self.t0 is None:
                self.t0 = t
            self.ekf.step(w_b, a_b, dt)

            self.frame_id = frame_id
            self.a_opt, self.w_opt = a_opt, w_opt
            self.a_body, self.w_body = a_b, w_b
            self.count += 1
            self.last_wall = time.monotonic()

            now = time.monotonic()
            if self._hz_t is None:
                self._hz_t, self._hz_n = now, 0
            self._hz_n += 1
            if now - self._hz_t >= 0.5:
                self.hz = self._hz_n / (now - self._hz_t)
                self._hz_t, self._hz_n = now, 0

            # 画面の時系列グラフ用。200Hz 全部は要らないので 50Hz に間引く
            if self.ekf.ready and self.count % 4 == 0:
                r, p, y = R_to_rpy(quat_to_R(self.ekf.q))
                self.trace.append([round(t - self.t0, 3),
                                   round(math.degrees(r), 2),
                                   round(math.degrees(p), 2),
                                   round(math.degrees(y), 2)])

    def snapshot(self) -> dict:
        with self.lock:
            alive = (time.monotonic() - self.last_wall) < 1.0 if self.count else False
            if not self.ekf.ready:
                return {'ok': 1, 'ready': 0, 'alive': alive, 'n': self.count,
                        'hz': round(self.hz, 2), 'error': self.error,
                        'frame_id': self.frame_id}
            R = quat_to_R(self.ekf.q)
            r, p, y = R_to_rpy(R)
            ar, ap = tilt_from_accel(self.a_body)
            amag = float(np.linalg.norm(self.a_body))
            # 加速度計が示す「上」を世界座標に置き直したもの。静止して推定が
            # 合っていれば、これは世界 z 軸（真上）にほぼ一致する。
            up_w = R @ (self.a_body / (amag or 1.0))
            # P が持っているのは「機体 x/y/z 軸まわりの姿勢誤差」で、
            # ロール・ピッチ・ヨーの誤差そのものではない（ZYX の各角は別々の軸
            # まわりなので、傾くとずれる）。軸まわりの σ をそのまま出しつつ、
            # ヨーの当てにならなさは **鉛直まわり** の σ として別に出す。
            # これが観測不能な方向で、時間とともに素直に増えていく。
            P_th = self.ekf.P[0:3, 0:3]
            sd = np.sqrt(np.clip(np.diag(P_th), 0.0, None))
            v = R.T @ np.array([0.0, 0.0, 1.0])          # 機体系で見た鉛直
            sd_head = math.sqrt(max(float(v @ P_th @ v), 0.0))
            return {
                'ok': 1, 'ready': 1, 'alive': alive,
                'n': self.count, 'hz': round(self.hz, 2),
                't': round((self.t_last - self.t0), 2),
                'frame_id': self.frame_id,
                'error': self.error,
                'R': [round(v, 6) for v in R.flatten().tolist()],
                'rpy': [round(math.degrees(v), 3) for v in (r, p, y)],
                'rpy_accel': [round(math.degrees(ar), 3), round(math.degrees(ap), 3)],
                'bias': [round(v, 5) for v in self.ekf.b.tolist()],
                'sigma_axis': [round(math.degrees(v), 3) for v in sd.tolist()],
                'sigma_heading': round(math.degrees(sd_head), 3),
                'a_opt': [round(v, 4) for v in self.a_opt.tolist()],
                'w_opt': [round(v, 5) for v in self.w_opt.tolist()],
                'a_body': [round(v, 4) for v in self.a_body.tolist()],
                'w_body': [round(v, 5) for v in self.w_body.tolist()],
                'a_mag': round(amag, 4),
                'up_world': [round(v, 4) for v in up_w.tolist()],
                'moving': int(abs(amag - G) > self.ekf.accel_gate),
                'gate_reject': self.ekf.gate_reject,
                'bias_clipped': self.ekf.bias_clipped,
                # 起動直後の過渡でも数回は当たる。ずっと警告を出しても意味がないので、
                # 「今まさに当たり続けているか」を別に出す（直近 2 秒 = 400 サンプル）。
                'bias_clipping': int(self.ekf.n - self.ekf.clip_at < 400),
                'bias_max': self.ekf.bias_max,
                'trace': list(self.trace),
            }


class RosReader(threading.Thread):
    """/camera/imu を購読して ImuState に流す。"""

    daemon = True

    def __init__(self, state: ImuState, topic: str):
        super().__init__()
        self.state = state
        self.topic = topic
        self._quit = threading.Event()

    def stop(self):
        """spin を抜けさせる。ノードの後始末は spin と同じスレッドでやる。

        フラグ名を ``_stop`` にしてはいけない。``threading.Thread`` が内部で
        ``self._stop()`` を呼ぶので、同名の Event で潰すとスレッド終了時に
        TypeError で落ちる。
        """
        self._quit.set()
        self.join(timeout=2.0)

    def run(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from rclpy.signals import SignalHandlerOptions
            from sensor_msgs.msg import Imu
        except ImportError as exc:
            self.state.error = ('rclpy が import できない（%s）。'
                                'source /opt/ros/jazzy/setup.bash を先に。' % exc)
            return

        st = self.state

        class Sub(Node):
            def __init__(self):
                super().__init__('imu3d_viz')
                # /camera/imu は SENSOR_DATA(BEST_EFFORT)。RELIABLE で購読すると
                # QoS が合わず 1 つも受け取れない（realsense_bringup/README.md）。
                self.create_subscription(Imu, topic_name, self.cb,
                                         qos_profile_sensor_data)

            def cb(self, msg):
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                a = np.array([msg.linear_acceleration.x,
                              msg.linear_acceleration.y,
                              msg.linear_acceleration.z])
                w = np.array([msg.angular_velocity.x,
                              msg.angular_velocity.y,
                              msg.angular_velocity.z])
                st.feed(t, msg.header.frame_id, a, w)

        topic_name = self.topic
        # シグナルは受け取らない。既定だと rclpy が SIGINT/SIGTERM を横取りして
        # context だけ畳むので、メインスレッドの serve_forever が生き残り、
        # **Ctrl-C でも kill でもプロセスが終わらなくなる**。
        # 停止はメインスレッド側（KeyboardInterrupt と既定の SIGTERM）に任せ、
        # この購読スレッドは daemon なので道連れで終わる。
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = Sub()
        try:
            # spin() だと停止要求で抜けられない。抜けないまま daemon スレッドごと
            # 落とすと、C++ 側の executor が生きたままで終了時に
            # 「terminate called without an active exception」が出る。
            while rclpy.ok() and not self._quit.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception as exc:                                # noqa: BLE001
            self.state.error = 'rclpy が止まった（%s）' % exc
        finally:
            node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:                                   # noqa: BLE001
                pass


class DemoReader(threading.Thread):
    """実機なしの確認用。光学フレームの生値を合成して同じ経路に流す。

    ロール・ピッチ・ヨーを既知の値で振り、そこから加速度計とジャイロの読みを
    逆算する。EKF と軸の載せ替えが正しければ、画面の数字がここで作った値に戻る。
    """

    daemon = True

    def __init__(self, state: ImuState, rate=200.0):
        super().__init__()
        self.state = state
        self.rate = rate

    def run(self):
        dt = 1.0 / self.rate
        t = 0.0
        bias_b = np.array([0.004, -0.002, 0.003])   # ジャイロのバイアス（模擬）
        Minv = M_OPT2BODY.T                         # 直交なので転置が逆
        rng = np.random.default_rng(0)
        while True:
            roll = math.radians(25.0) * math.sin(2*math.pi*t/11.0)
            pitch = math.radians(18.0) * math.sin(2*math.pi*t/7.0 + 1.0)
            yaw = math.radians(40.0) * math.sin(2*math.pi*t/17.0)
            h = 1e-4
            q0 = rpy_to_quat(roll, pitch, yaw)
            q1 = rpy_to_quat(
                roll + math.radians(25.0)*(math.sin(2*math.pi*(t+h)/11.0)
                                           - math.sin(2*math.pi*t/11.0)),
                pitch + math.radians(18.0)*(math.sin(2*math.pi*(t+h)/7.0+1.0)
                                            - math.sin(2*math.pi*t/7.0+1.0)),
                yaw + math.radians(40.0)*(math.sin(2*math.pi*(t+h)/17.0)
                                          - math.sin(2*math.pi*t/17.0)))
            # 角速度は R^T Ṙ から。差分で作って body 系に落とす
            dR = (quat_to_R(q1) - quat_to_R(q0)) / h
            Wx = quat_to_R(q0).T @ dR
            w_b = np.array([Wx[2, 1], Wx[0, 2], Wx[1, 0]]) + bias_b
            w_b = w_b + rng.normal(0.0, 0.003, 3)
            a_b = quat_to_R(q0).T @ np.array([0.0, 0.0, G])
            a_b = a_b + rng.normal(0.0, 0.013, 3)
            self.state.feed(t, 'camera_imu_optical_frame (demo)',
                            Minv @ a_b, Minv @ w_b)
            t += dt
            time.sleep(dt)


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------
STATE: ImuState | None = None


class Handler(BaseHTTPRequestHandler):
    server_version = 'imu3d'

    def log_message(self, fmt, *args):
        pass

    def _send(self, body: bytes, ctype: str, code: int = 200):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(json.dumps(obj).encode('utf-8'), 'application/json', code)

    def do_GET(self):
        u = urlparse(self.path)
        try:
            if u.path == '/favicon.ico':
                self._send(b'', 'image/x-icon', 204)
            elif not u.path.startswith('/api/'):
                self._send(PAGE.read_bytes(), 'text/html; charset=utf-8')
            elif u.path == '/api/state':
                self._json(STATE.snapshot())
            elif u.path == '/api/reset':
                with STATE.lock:
                    STATE.ekf.reset()
                    STATE.trace.clear()
                self._json({'ok': 1})
            elif u.path == '/api/zeroyaw':
                with STATE.lock:
                    STATE.ekf.zero_yaw()
                self._json({'ok': 1})
            else:
                self._json({'ok': 0, 'error': '知らない API: %s' % u.path}, 404)
        except Exception as exc:                                # noqa: BLE001
            self._json({'ok': 0, 'error': '%s: %s' % (type(exc).__name__, exc)}, 500)


def ssh_addr() -> str | None:
    """ssh でログインしている場合、こちら側（サーバ側）の IP。"""
    f = os.environ.get('SSH_CONNECTION', '').split()
    return f[2] if len(f) >= 3 else None


def local_addrs():
    out = []
    a = ssh_addr()
    if a:
        out.append(('ssh', a))
    try:
        raw = subprocess.run(['ip', '-4', '-o', 'addr', 'show'],
                             capture_output=True, text=True).stdout
        for line in raw.splitlines():
            f = line.split()
            if len(f) > 3 and f[1] != 'lo':
                addr = f[3].split('/')[0]
                if addr not in [x for _, x in out]:
                    out.append((f[1], addr))
    except OSError:
        pass
    return out


def main() -> int:
    global STATE
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', type=int, default=8104)
    ap.add_argument('--bind', default=None,
                    help='既定は ssh 元から見えるこの機体の IP（SSH_CONNECTION）。'
                         'ssh 経由でなければ 0.0.0.0')
    ap.add_argument('--topic', default='/camera/imu')
    ap.add_argument('--demo', action='store_true', help='実機なしで表示だけ確かめる')
    args = ap.parse_args()

    bind = args.bind or ssh_addr() or '0.0.0.0'

    STATE = ImuState()
    reader = DemoReader(STATE) if args.demo else RosReader(STATE, args.topic)
    reader.start()

    srv = ThreadingHTTPServer((bind, args.port), Handler)
    print('=' * 70)
    print('IMU 姿勢 3D ビジュアライザ（EKF・推定するだけで publish はしない）')
    print('   購読: %s' % ('--demo（合成データ）' if args.demo else args.topic))
    print('   bind: %s:%d' % (bind, args.port))
    for name, addr in local_addrs():
        print('   %-6s http://%s:%d/' % (name, addr, args.port))
    print('   停止は Ctrl-C')
    print('=' * 70, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\n停止')
    finally:
        srv.server_close()
        if hasattr(reader, 'stop'):
            reader.stop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
