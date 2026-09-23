# -*- coding: utf-8 -*-
r"""opponent_viewer — 相手機の認識のテストラン。検出器をそのまま回してブラウザで見る。

    # 実機のカメラで (カメラごと起動。サーボには触れない)
    ros2 launch roboone_perception opponent_testrun.launch.py camera:=true

    # カメラが既に上がっているなら
    ros2 run roboone_perception opponent_viewer --ros-args \\
        --params-file src/roboone_perception/config/opponent_detector.yaml

    # 実機なしで画面と判定だけ確かめる (合成シーン。ROS も要らない)
    ros2 run roboone_perception opponent_viewer --demo

起動時に表示される URL (既定 8105 番) を手元の PC のブラウザで開く。
見るもの・確かめる順は docs/相手機の認識.md §9。

===========================================================================
これは何か
===========================================================================
**opponent_detector ノードそのもの**に HTTP の口を付けたもの。別実装の検出器を
持たないので、ここで見えているものが本番で /opponent に出るものと同じになる。
/opponent と /ring_edge も通常どおり publish するので、行動層を一緒に上げて
試すこともできる。**opponent_detector と同時には上げないこと** (/opponent が二重になる)。

転倒の判定は行動層の FallenDetector (roboone_behavior) を既定のパラメータで回す。
「倒れているか」の 1 フレームの判断は検出器がクラスタ上端 z_top の絶対値で済ませて
いるので (opponent_detector.yaml の match.fallen_top_max / robot_top_max)、ここで
回しているのは時間の門 (T_down / T_up) だけ。画面の「判定をやり直す」はその門を
リセットする。

既定ではサーボへ何も書かない。motion ノードも要らない。

===========================================================================
--home: 立たせた状態で見る (★トルクが入る)
===========================================================================
カメラは胴体に付いているので、本番と同じ高さ・向きで見るには機体をホーム姿勢で
立たせる必要がある。``--home`` を付けたときだけ、motion ノードへ teleop と同じ順で

    /cmd_motion "home"  →  少し置いて  /estop false

を送る (launch の home:=true が motion ノードも一緒に上げ、武装の補間時間
torque_on_time を home_time 秒に延ばすので、実測姿勢からゆっくりホームへ移る)。
画面に「脱力」「ホームへ」のボタンが出る。終了時は /estop true を置いていく。
``--home`` が無ければ /estop にも /cmd_motion にも publisher を作らない。
"""

import argparse
import base64
import csv
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np

from .detect import clusters as cl
from .detect import DetectorParams, Intrinsics, RingDetector
from .detect import grid as g
from .detect.polar import fov_half_deg

try:                                    # 行動層が無くても表示だけは動かす
    from roboone_behavior.behavior import (BehaviorParams, FallenDetector,
                                           KIND_ROBOT_FALLEN, KIND_ROBOT_STANDING)
except ImportError:                     # pragma: no cover
    BehaviorParams = FallenDetector = None
    KIND_ROBOT_FALLEN = KIND_ROBOT_STANDING = None

#: 検出器の種別 (文字列) → Opponent.kind の値。行動層へ渡すときに挟む
_KIND_MSG = {cl.ROBOT_STANDING: KIND_ROBOT_STANDING,
             cl.ROBOT_FALLEN: KIND_ROBOT_FALLEN}

#: 画面に出す日本語
_KIND_JA = {cl.ROBOT_STANDING: '立位', cl.ROBOT_FALLEN: '転倒',
            cl.HUMAN: '人・什器', cl.NOISE: '低すぎ'}

#: 俯瞰図のセルの分類。値が大きいほど上に塗る (viz/opponent_view.html の CLASS_COLORS)。
#: 塊は**種別ごとに 3 色**に分ける (2026-09-23)。セルの種別は、その柱の一番上の
#: ボクセルが属する塊のもの (DetectionResult.obj_labels)
_INTERIOR, _RING, _EDGE, _ABOVE = 1, 2, 3, 4
_OBJ_OTHER, _OBJ_FALLEN, _OBJ_STANDING, _OUTSIDE = 5, 6, 7, 8

#: 塊の種別 → 俯瞰図の分類。ここに無い種別 (人・什器、低すぎ) は _OBJ_OTHER
_KIND_CLASS = {cl.ROBOT_STANDING: _OBJ_STANDING, cl.ROBOT_FALLEN: _OBJ_FALLEN}

_STATUS_JA = {
    'OK': '相手を捉えている',
    'NO_OPPONENT': '相手が見えない',
    'ATTITUDE_STALE': '姿勢が劣化 (床の面あてはめが通らない)',
    'RING_LOST': 'リング面が取れない',
    'NO_DEPTH': '深度が来ていない',
}


def _num(v, nd=3):
    """JSON に載せる数。NaN / inf / None は null にする。"""
    if v is None:
        return None
    v = float(v)
    return round(v, nd) if math.isfinite(v) else None


class ViewState:
    """検出結果を画面用の辞書にして持つ。検出のスレッドが書き、HTTP のスレッドが読む。"""

    #: 深度画像を詰め直す最短の間隔 [s]。bag のビューアは短くする
    depth_period = 0.25

    def __init__(self, params, source, view_hz=10.0, log_path=None):
        self.p = params
        self.source = source
        self.lock = threading.Lock()
        self.state = {'ready': False, 'source': source}
        self.depth = {'ready': False}
        self.events = []
        self.min_period = 1.0 / max(1.0, view_hz)
        self.last_pack = 0.0
        self.last_depth_pack = 0.0
        self.t0 = time.monotonic()
        self.frame_times = []
        self.n_frames = 0
        self.prev_status = None
        self.prev_fallen = None
        self.intr = None

        self.fall = None
        self.bparams = None
        if FallenDetector is not None:
            self.bparams = BehaviorParams()
            self.fall = FallenDetector(self.bparams.tune, self.bparams.robot)

        #: 基準姿勢の取り直し。入力の側 (検出器ノード / 合成シーン) が差し込む
        self.reset_cb = None
        self.reset_status_cb = lambda: ''

        self.csv = None
        if log_path:
            self._log_file = open(log_path, 'w', newline='')
            self.csv = csv.writer(self._log_file)
            self.csv.writerow(['t', 'status', 'x', 'y', 'range', 'bearing_deg',
                               'top', 'width', 'kind', 'extrapolated', 'fallen',
                               'n_clusters', 'n_intruding',
                               'ring_area', 'cam_height_meas', 'proc_ms'])

    # ------------------------------------------------------------ 操作
    def recalibrate(self):
        with self.lock:
            if self.fall is not None:
                self.fall.reset()
            self._event('転倒判定の時間の門をリセットする')

    def reset_attitude(self, why='画面のボタン'):
        """基準姿勢を取り直す。直前までの判定は当てにならないので転倒判定も戻す。"""
        if self.reset_cb is None:
            return False
        self.reset_cb(why)
        with self.lock:
            if self.fall is not None:
                self.fall.reset()
            self.prev_fallen = None
            self._event('基準姿勢を取り直す (%s)。転倒判定もリセットする' % why)
        return True

    def _event(self, text):
        self.events.append({'t': round(time.monotonic() - self.t0, 1), 'text': text})
        del self.events[:-40]

    # ------------------------------------------------------------ 1 フレーム
    def feed(self, res, depth, scale, dt, intr, truth=None):
        """検出器の 1 フレームを受け取る。検出のスレッドから呼ばれる。"""
        now = time.monotonic()
        with self.lock:
            self.intr = intr
            self.n_frames += 1
            self.frame_times.append(now)
            del self.frame_times[:-60]

            b = self.p.body
            opp = None
            if res.position is not None:
                x = res.position[0] + b.cam_offset_x
                y = res.position[1] + b.cam_offset_y
                opp = {'x': x, 'y': y, 'range': math.hypot(x, y),
                       'bearing': math.atan2(y, x)}

            # --- 転倒判定。行動層と同じ規則: 外挿中と OK 以外は観測として渡さない
            fresh = res.status == 'OK' and opp is not None and not res.extrapolated
            fallen = None
            if self.fall is not None:
                fallen = self.fall.step(
                    _KIND_MSG.get(res.kind) if fresh else None,
                    res.width if fresh else None,
                    opp['range'] if opp else None,
                    max(float(dt), 1e-3))

            if res.status != self.prev_status:
                self._event('検出: %s → %s' % (self.prev_status or '-', res.status))
                self.prev_status = res.status
            if fallen is not None and fallen != self.prev_fallen:
                if self.prev_fallen is not None or fallen:
                    self._event('相手が%s' % ('倒れた' if fallen else '立ち上がった'))
                self.prev_fallen = fallen

            if self.csv is not None:
                self.csv.writerow([
                    '%.3f' % (now - self.t0), res.status,
                    *(['%.3f' % v for v in (opp['x'], opp['y'], opp['range'],
                                            math.degrees(opp['bearing']))]
                      if opp else ['', '', '', '']),
                    _num(res.top_height), _num(res.width), res.kind or '',
                    int(res.extrapolated),
                    '' if fallen is None else int(fallen),
                    len(res.clusters), sum(1 for c in res.clusters if c.intruding),
                    _num(res.ring_area),
                    _num(-res.ring_height) if res.ring_height is not None else '',
                    _num(res.timings.get('total', 0.0) * 1e3, 2)])

            if now - self.last_pack >= self.min_period:
                self.last_pack = now
                self.state = self._pack(res, opp, fallen, fresh, truth)
            if depth is not None and now - self.last_depth_pack >= self.depth_period:
                self.last_depth_pack = now
                self.depth = self._pack_depth(depth, scale)

    def no_depth(self):
        with self.lock:
            if self.prev_status != 'NO_DEPTH':
                self._event('深度が来ていない')
                self.prev_status = 'NO_DEPTH'
            self.state = dict(self.state, status='NO_DEPTH',
                              status_ja=_STATUS_JA['NO_DEPTH'], opponent=None)

    # ------------------------------------------------------------ 詰め替え
    def _fps(self):
        ft = self.frame_times
        if len(ft) < 2 or ft[-1] <= ft[0]:
            return 0.0
        return (len(ft) - 1) / (ft[-1] - ft[0])

    @staticmethod
    def _object_layers(res):
        """塊のセルを種別ごとに分けて (マスク, 分類) で返す。

        セルの種別は、その柱の一番上のボクセルが属する塊のものにする。腕の下に
        相手がいるセルは「腕」の色になる。見えているとおりの塗り方で、門 4
        (上に物が乗った塊は相手にしない) が効いている理由がそのまま読める。
        """
        if res.obj_labels is None or not res.clusters:
            return [(res.obj_mask, _OBJ_OTHER)] if res.obj_mask is not None else []
        out = []
        for code in (_OBJ_OTHER, _OBJ_FALLEN, _OBJ_STANDING):
            ids = [c.label for c in res.clusters
                   if _KIND_CLASS.get(c.kind, _OBJ_OTHER) == code]
            if ids:
                out.append((np.isin(res.obj_labels, ids), code))
        return out

    def _pack(self, res, opp, fallen, fresh, truth):
        t = self.p.tune
        grid = None
        if res.ring_mask is not None:
            cls = np.zeros(res.ring_mask.shape, dtype=np.uint8)
            layers = [(res.interior_mask, _INTERIOR),
                      (res.ring_mask, _RING),
                      (g.boundary(res.ring_mask), _EDGE),
                      (res.above_mask, _ABOVE)]
            layers += self._object_layers(res)
            layers.append((res.outside_mask, _OUTSIDE))
            for mask, code in layers:
                if mask is not None:
                    cls[mask] = code
            grid = {'nu': int(cls.shape[0]), 'nv': int(cls.shape[1]),
                    'cell': t.cell, 'u_min': -t.grid_back, 'v_min': -t.grid_side,
                    'data': base64.b64encode(cls.tobytes()).decode('ascii')}

        clusters = []
        for c in sorted(res.clusters, key=lambda c: c.radius):
            clusters.append({
                'fwd': _num(c.fwd), 'left': _num(c.left), 'range': _num(c.radius),
                'top': _num(c.top_height), 'width': _num(c.width),
                'points': c.n_points, 'cells': c.n_cells,
                'outside_cells': c.outside_cells,
                'selected': c is res.selected,
                'kind': c.kind, 'kind_ja': _KIND_JA.get(c.kind, ''),
                'reason': cl.reject_reason(c, self.p.match, t)})

        fall = None
        if self.fall is not None:
            f = self.fall
            fall = {'fallen': bool(fallen),
                    'kind': res.kind, 'kind_ja': _KIND_JA.get(res.kind, ''),
                    'evidence': dict(f.evidence) if fresh else None,
                    'below': _num(f._below, 2), 'above': _num(f._above, 2),
                    'fallen_time': self.bparams.tune.fallen_time,
                    'stand_time': self.bparams.tune.stand_time}

        tilt = None
        if res.up is not None:
            # 鉛直 u (カメラ座標 x右 y下 z前) から、カメラの俯角とロール
            ux, uy, uz = (float(v) for v in res.up)
            tilt = {'pitch_deg': _num(math.degrees(math.atan2(-uz, -uy)), 1),
                    'roll_deg': _num(math.degrees(math.atan2(ux, -uy)), 1)}

        out = {
            'ready': True, 'source': self.source,
            't': round(time.monotonic() - self.t0, 2),
            'frames': self.n_frames, 'fps': _num(self._fps(), 1),
            'proc_ms': _num(res.timings.get('total', 0.0) * 1e3, 1),
            'timings_ms': {k: _num(v * 1e3, 2) for k, v in res.timings.items()},
            'status': res.status, 'status_ja': _STATUS_JA.get(res.status, res.status),
            'opponent': None, 'fall': fall, 'clusters': clusters, 'grid': grid,
            'cliff': ([_num(v) for v in res.cliff] if res.cliff is not None else None),
            'cliff_half_deg': t.edge_half_fov_deg,
            'fov_half_deg': _num(fov_half_deg(self.intr), 1) if self.intr else None,
            'seed_window': ([_num(v) for v in res.seed_window]
                            if res.seed_window else None),
            'seed_half_width': t.seed_half_width,
            'ring_area': _num(res.ring_area, 2),
            'cam_height_meas': (_num(-res.ring_height)
                                if res.ring_height is not None else None),
            'cam_height_param': self.p.body.cam_height,
            'cam_pitch_param': self.p.body.cam_pitch_deg,
            'plane_resid_mm': _num(res.plane_resid * 1e3, 1),
            'plane_angle_deg': _num(res.plane_angle_deg, 1),
            'stale_frames': int(res.stale_frames),
            'reset_status': self.reset_status_cb(),
            'tilt': tilt, 'n_points': res.n_points,
            'limits': {'top_min': self.p.match.obj_top_min,
                       'fallen_top_max': self.p.match.fallen_top_max,
                       'top_max': self.p.match.robot_top_max,
                       'width_min': self.p.match.obj_width_min,
                       'width_max': self.p.match.obj_width_max},
            'events': list(self.events), 'truth': truth,
        }
        if opp is not None:
            out['opponent'] = {
                'x': _num(opp['x']), 'y': _num(opp['y']),
                'range': _num(opp['range']),
                'bearing_deg': _num(math.degrees(opp['bearing']), 1),
                'top': _num(res.top_height), 'width': _num(res.width),
                'height': _num(res.height),
                'vx': _num(res.velocity[0]), 'vy': _num(res.velocity[1]),
                'extrapolated': bool(res.extrapolated)}
        return out

    @staticmethod
    def _pack_depth(depth, scale, max_w=212, z_near=0.15, z_far=4.0):
        step = max(1, int(math.ceil(depth.shape[1] / max_w)))
        d = depth[::step, ::step].astype(np.float32) * float(scale)
        valid = d > 0
        v = np.clip((d - z_near) / (z_far - z_near), 0.0, 1.0)
        img = np.where(valid, 1 + np.round(v * 254), 0).astype(np.uint8)
        return {'ready': True, 'w': int(img.shape[1]), 'h': int(img.shape[0]),
                'near': z_near, 'far': z_far,
                'data': base64.b64encode(img.tobytes()).decode('ascii')}

    def snapshot(self):
        with self.lock:
            return self.state

    def depth_snapshot(self):
        with self.lock:
            return self.depth


# ==========================================================================
# ホーム姿勢へ立たせる (--home のときだけ作る)
# ==========================================================================
class HomeControl:
    """motion ノードへ home → /estop false を送る。teleop の Options 長押しと同じ順。

    motion は /cmd_motion を 1 回受けるまで武装しない (require_home_before_arm)。
    /estop は latched (TRANSIENT_LOCAL) で出さないと motion の購読と QoS が合わず
    1 通も届かない。起動直後にまず /estop true を置き、motion が上がったのを
    /motion/state で確かめてから delay 秒待って立たせる。
    """

    def __init__(self, node, delay=3.0, gap=0.3, on_standing=None):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool, String
        self._Bool, self._String = Bool, String
        self.node = node
        self.delay = float(delay)
        self.gap = float(gap)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_estop = node.create_publisher(Bool, '/estop', latched)
        self.pub_motion = node.create_publisher(String, '/cmd_motion', 10)
        node.create_subscription(String, '/motion/state', self._on_state, latched)
        self.motion_state = None
        self.phase = 'motion ノードを待っている'
        self.lock = threading.Lock()
        self._t_seen = None
        self._t_home = None          # home を送った時刻。gap 後に /estop false
        self._auto_done = False
        #: ホーム姿勢で立ち終えた (HOLD になった) ときに 1 回呼ぶ。基準姿勢の取り直し用。
        #: 脱力した姿勢から立ち上がる間は床が見えず、鉛直の推定が外れたままになる
        self.on_standing = on_standing
        self._want_standing = False
        self.pub_estop.publish(Bool(data=True))
        node.create_timer(0.1, self._tick)

    def _now(self):
        return time.monotonic()

    def _on_state(self, msg):
        with self.lock:
            if self.motion_state is None:
                self._t_seen = self._now()
            self.motion_state = msg.data
            fire = self._want_standing and msg.data == 'HOLD'
            if fire:
                self._want_standing = False
        if fire and self.on_standing is not None:
            self.on_standing()

    def _tick(self):
        with self.lock:
            now = self._now()
            if not self._auto_done and self._t_seen is not None:
                left = self.delay - (now - self._t_seen)
                if left > 0:
                    self.phase = 'あと %.0f 秒でトルクを入れてホームへ' % math.ceil(left)
                else:
                    self._auto_done = True
                    self._send_home(now)
            if self._t_home is not None and now - self._t_home >= self.gap:
                self._t_home = None
                self.pub_estop.publish(self._Bool(data=False))
                self.phase = 'home → /estop false を送った (実際の状態は左の motion)'
                self.node.get_logger().warn('★ /estop false を送った。トルクが入る')

    def _send_home(self, now):
        self.pub_motion.publish(self._String(data='home'))
        self._want_standing = True
        self._t_home = now
        self.phase = 'home を送った'
        self.node.get_logger().warn('★ /cmd_motion home を送った。続けてトルクを入れる')

    def home(self):
        with self.lock:
            self._auto_done = True
            self._send_home(self._now())

    def relax(self):
        with self.lock:
            self._auto_done = True          # 起動直後の秒読み中に押されたら立たせない
            self._want_standing = False
            self._t_home = None
            self.pub_estop.publish(self._Bool(data=True))
            self.phase = '脱力した'
            self.node.get_logger().warn('/estop true を送った。脱力')

    def snapshot(self):
        with self.lock:
            return {'state': self.motion_state, 'phase': self.phase}


# ==========================================================================
# 合成シーン (--demo)
# ==========================================================================
class DemoFeeder(threading.Thread):
    """実機なしで検出器へ合成の深度を流す。ROS も要らない。

    40 秒でひと回りする筋書き (水平付け・高さ 0.40 m のカメラ):

         0-12 s  相手が 1.6 m 先から 0.45 m まで歩いてくる (足元の死角に入っても追う)
        12-20 s  相手が横倒しになる → 0.7 s ほどで「倒れている」
        20-27 s  立ち上がる → 0.5 s ほどで「立っている」
        27-40 s  相手が消え、左の縁の外に立つ人がリングの上へ腕を伸ばす
                 → 塊は出るが「外から差し込んでいる」で相手にしない

    左の縁の外には常に人が立っている (赤 = リングの外の物)。
    """

    PERIOD = 40.0

    def __init__(self, view, params, rate=12.0):
        super().__init__(daemon=True)
        from .sim import scene as S
        self.S = S
        self.view = view
        self.params = params
        self.rate = rate
        self.intr = Intrinsics(424, 240, 212.0, 212.0, 212.0, 120.0)
        self.det = RingDetector(params)
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def _scene(self, t):
        S = self.S
        boxes = [S.Box(1.3, 1.25, 0.45, 0.30, -0.34, 1.35)]        # 縁の外に立つ人
        truth = '相手なし'
        if t < 12.0:
            k = t / 12.0
            x, y = 1.6 - 1.15 * k, -0.4 + 0.5 * k
            boxes.append(S.Box(x, y, 0.25, 0.15, 0.0, 0.45))
            truth = '立っている相手が近づく (%.2f m)' % math.hypot(x, y)
        elif t < 20.0:
            boxes.append(S.Box(0.95, 0.1, 0.50, 0.28, 0.0, 0.14))
            truth = '相手が横倒し'
        elif t < 27.0:
            boxes.append(S.Box(0.95, 0.1, 0.25, 0.15, 0.0, 0.45))
            truth = '相手が立ち上がった'
        else:
            reach = min(1.0, (t - 27.0) / 3.0)
            boxes.append(S.Box(1.3, 1.10 - 0.35 * reach, 0.10 + 0.70 * reach,
                               0.08, 0.30, 0.38))
            truth = '人が外から腕を差し込む (相手ではない)'
        return S.Scene(center=(0.6, -1.0), boxes=boxes), truth

    def run(self):
        b = self.params.body
        dt = 1.0 / self.rate
        t0 = time.monotonic()
        k = 0
        while not self._stop.is_set():
            t = (time.monotonic() - t0) % self.PERIOD
            scene, truth = self._scene(t)
            roll = 1.2 * math.sin(2 * math.pi * (time.monotonic() - t0) / 0.6)
            with np.errstate(invalid='ignore', divide='ignore'):
                depth = self.S.render(scene, self.intr, cam_height=b.cam_height,
                                      pitch_deg=b.cam_pitch_deg, roll_deg=roll,
                                      noise=0.0015, hole_rate=0.01, seed=k)
            res = self.det.step(depth, self.intr, dt)
            self.view.feed(res, depth, 0.001, dt, self.intr, truth=truth)
            k += 1
            lag = (k * dt) - (time.monotonic() - t0)
            if lag > 0:
                time.sleep(lag)
            else:
                k = int((time.monotonic() - t0) / dt)


# ==========================================================================
# HTTP
# ==========================================================================
VIEW = None
HOME = None           # HomeControl。--home のときだけ


def _html_path():
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [os.path.join(here, '..', 'viz', 'opponent_view.html')]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.insert(0, os.path.join(
            get_package_share_directory('roboone_perception'), 'viz',
            'opponent_view.html'))
    except Exception:                                           # noqa: BLE001
        pass
    for c in cands:
        if os.path.exists(c):
            return c
    raise FileNotFoundError('opponent_view.html が見つからない: %s' % cands)


class Handler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):
        pass

    def _send(self, body, ctype, code=200):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(json.dumps(obj, ensure_ascii=False).encode('utf-8'),
                   'application/json; charset=utf-8', code)

    def do_GET(self):
        path = self.path.split('?')[0]
        try:
            if path in ('/', '/index.html'):
                with open(_html_path(), 'rb') as f:     # 毎回読む。直したらすぐ反映
                    self._send(f.read(), 'text/html; charset=utf-8')
            elif path == '/state':
                st = VIEW.snapshot()
                if HOME is not None:
                    st = dict(st, motion=HOME.snapshot())
                self._json(st)
            elif path == '/depth':
                self._json(VIEW.depth_snapshot())
            else:
                self._json({'ok': 0, 'error': '知らない API: %s' % path}, 404)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        path = self.path.split('?')[0]
        if path == '/recal':
            VIEW.recalibrate()
            self._json({'ok': 1})
        elif path == '/reset_attitude':
            self._json({'ok': int(VIEW.reset_attitude())})
        elif path in ('/relax', '/home') and HOME is None:
            self._json({'ok': 0, 'error': '--home を付けて起動していない'}, 403)
        elif path == '/relax':
            HOME.relax()
            self._json({'ok': 1})
        elif path == '/home':
            HOME.home()
            self._json({'ok': 1})
        else:
            self._json({'ok': 0, 'error': '知らない API: %s' % path}, 404)


def _ssh_addr():
    f = os.environ.get('SSH_CONNECTION', '').split()
    return f[2] if len(f) >= 3 else None


def _local_addrs():
    out = []
    a = _ssh_addr()
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


def _banner(args, bind, source):
    print('=' * 72)
    if getattr(args, 'home', False):
        print('相手機の認識 テストラン (検出器 + 転倒判定)')
        print('   ★★ --home: motion が上がって %.0f 秒後にトルクを入れ、ホーム姿勢へ立たせる ★★'
              % args.home_delay)
        print('   ★★ 機体を支えておくこと。止めるのは画面の「脱力」か Ctrl-C ★★')
    else:
        print('相手機の認識 テストラン (検出器 + 転倒判定。サーボには触れない)')
    print('   入力: %s' % source)
    if FallenDetector is None:
        print('   ★ roboone_behavior が見つからないので転倒判定は出ない')
    if bind == '0.0.0.0':
        for name, addr in _local_addrs():
            print('   %-6s http://%s:%d/' % (name, addr, args.port))
    else:
        print('   http://%s:%d/' % (bind, args.port))
    if args.log:
        print('   記録: %s' % args.log)
    print('   停止は Ctrl-C')
    print('=' * 72, flush=True)


def main(argv=None):
    global VIEW, HOME
    ap = argparse.ArgumentParser(
        description='相手機の認識のテストラン (docs/相手機の認識.md §9)')
    ap.add_argument('--port', type=int, default=8105)
    ap.add_argument('--bind', default=None,
                    help='既定は 0.0.0.0 (この機体のどの IP からでも開ける)')
    ap.add_argument('--demo', action='store_true',
                    help='実機なし。合成シーンを流す (ROS も要らない)')
    ap.add_argument('--log', default='', help='1 フレームごとの判定を残す CSV')
    ap.add_argument('--view-hz', type=float, default=10.0, help='画面の更新周期')
    ap.add_argument('--cam-height', type=float, default=0.40,
                    help='--demo のカメラ高さ [m]')
    ap.add_argument('--home', action='store_true',
                    help='★トルクが入る。motion ノードへ home → /estop false を送って'
                         'ホーム姿勢で立たせる (既定は送らない)')
    ap.add_argument('--no-home', dest='home', action='store_false',
                    help='launch から「送らない」を明示するための対')
    ap.add_argument('--home-delay', type=float, default=3.0,
                    help='motion ノードを確かめてからトルクを入れるまでの待ち [s]')
    raw = sys.argv[1:] if argv is None else list(argv)
    ros_args = []
    if '--ros-args' in raw:                       # launch が足す分は rclpy へ渡す
        i = raw.index('--ros-args')
        raw, ros_args = raw[:i], raw[i:]
    args = ap.parse_args(raw)
    bind = args.bind or '0.0.0.0'

    if args.demo and args.home:
        ap.error('--demo と --home は一緒に使えない (合成シーンで機体を立たせる意味が無い)')
    if args.demo:
        params = DetectorParams.from_flat({'body.cam_pitch_deg': 0.0,
                                           'body.cam_height': args.cam_height})
        VIEW = ViewState(params, '--demo (合成シーン)', args.view_hz,
                         args.log or None)
        feeder = DemoFeeder(VIEW, params)
        VIEW.reset_cb = lambda why: feeder.det.reset_reference(accel=None)
        VIEW.reset_status_cb = lambda: '合成シーン (取り付けの鉛直へ戻すだけ)'
        feeder.start()
        srv = ThreadingHTTPServer((bind, args.port), Handler)
        _banner(args, bind, VIEW.source)
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            print('\n停止')
        finally:
            feeder.stop()
            srv.server_close()
        return 0

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from .opponent_detector_node import OpponentDetectorNode, spin_detector

    rclpy.init(args=ros_args or None, signal_handler_options=SignalHandlerOptions.NO)
    node = OpponentDetectorNode()
    source = str(node.get_parameter('depth_topic').value)
    VIEW = ViewState(node.detector.p, source, args.view_hz, args.log or None)

    def hook(res, depth, scale, dt):
        VIEW.p = node.detector.p          # tune.* を実行時に変えても表示が追う
        VIEW.feed(res, depth, scale, dt, node.intr)

    node.result_hook = hook
    VIEW.reset_cb = node.request_reset
    VIEW.reset_status_cb = lambda: node.reset_status
    if args.home:
        HOME = HomeControl(
            node, delay=args.home_delay,
            on_standing=lambda: VIEW.reset_attitude('ホーム姿勢で立った (HOLD)'))

    def watchdog():
        if node.last_status == 'NO_DEPTH' or node.last_depth_wall is None:
            VIEW.no_depth()

    node.create_timer(0.5, watchdog)

    srv = ThreadingHTTPServer((bind, args.port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    _banner(args, bind, source)
    try:
        spin_detector(node)          # IMU を depth に飢えさせない (ノードと同じ回し方)
    except KeyboardInterrupt:
        print('\n停止')
    finally:
        # launch は子へ SIGINT を送り直すことがある。片付けの途中で 2 回目を受けて
        # トレースバックを出さないよう、ここから先は無視する
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if HOME is not None:
                HOME.relax()              # 脱力を置いていく (motion も終了時に脱力する)
            node.publish_lost()
        except Exception:                                       # noqa: BLE001
            pass
        srv.shutdown()
        srv.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
