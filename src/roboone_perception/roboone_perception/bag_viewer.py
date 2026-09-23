# -*- coding: utf-8 -*-
r"""opponent_bag_viewer — RealSense の録画 (.bag) を検出器に通して、塊の状態をブラウザで見る。

    # フォルダを渡すと中の .bag を全部読み、画面で切り替えられる
    ros2 run roboone_perception opponent_bag_viewer bag_0712/

    # 1 本だけ。途中から・止めた状態で開く
    ros2 run roboone_perception opponent_bag_viewer bag_0712/0712_field_640_640_30deg_imu.bag \
        --start 5 --paused

    # 画面なしで最後まで流して、状態と塊の数を 1 秒ごとに出す
    ros2 run roboone_perception opponent_bag_viewer bag_0712/ --batch

起動時に表示される URL (既定 8106 番。テストランの 8105 と並べて開ける) をブラウザで開く。
実機にもトピックにも触れない (ROS のノードも作らない)。

===========================================================================
何を流しているか
===========================================================================
**opponent_detector と同じ RingDetector** に、bag の depth と IMU をそのまま入れる。
パラメータは入っている opponent_detector.yaml (--config で変えられる)。ビューアの
画面もテストラン (opponent_viewer) と同じ HTML で、bag のときだけ次が増える:

  * 再生の操作 (再生・一時停止・コマ送り・シーク・速さ・bag の切り替え)。キーボードでも
    Space / ← → / Home
  * **深度画像に塊を画素ごとに重ねたもの。** 検出器が点ごとに振ったボクセルの塊の番号を
    そのまま塗る (pipeline の want_debug)。俯瞰図は柱ごとに一番上のボクセルしか
    見えないので、腕の下の相手のような「3 次元で分かれた塊」はこちらで見る
  * 側面図 (前方距離 × 高さ)。高さの境界 3 本と一緒に、塊ごとの点を描く
  * 塊の色を「種別」と「塊ごと」で切り替える

===========================================================================
カメラの取り付けは bag ごとに測る (IMU と床の平面から)
===========================================================================
bag は手持ちや三脚で録ったもので、カメラの高さも俯角も機体の設定
(body.cam_height = 0.365 / cam_pitch_deg = 0) とは違う。検出器は

  * 鉛直 u の初期値が床の法線から 12 度以内でないと、面あてはめの門で弾き続けて戻れない
  * body.cam_height は高さヒストグラムの窓 (±0.25 m) の中心

なので、そのままでは床を掴めない。そこで**最初のフレームで取り付けを測ってから**流す:

  1. IMU がある bag は、そのフレームの前後 0.25 秒の加速度の平均から鉛直を出す
  2. その鉛直を手がかりに、近く (3 m 以内) の点から床の平面を RANSAC で探す。
     手がかりから 15 度 (IMU なしは 40 度) 以内の法線に限る。IMU が無ければ
     ファイル名の「30deg」「holizontal」から俯角の見当をつける
  3. 平面が取れたら、**その法線を鉛直、原点からの距離をカメラ高さ**にする
     (取れなければ IMU の鉛直と設定の高さのまま)

測った俯角とカメラ高さで body.cam_pitch_deg / cam_height を置き換えた検出器を作り、
鉛直をその法線に置いてから流し始める。以降の姿勢は検出器そのもの (ジャイロで運び、
床の法線で毎フレーム引き戻す) で、ここは手を出さない。IMU の無い bag はジャイロの
予測が無く、床の法線だけで追うことになる。

**取り直し。** RING_LOST / ATTITUDE_STALE が既定 15 枚 (0.5 秒) 続いたら、上の 1〜3 を
そのフレームでやり直す (--no-reacquire で切る)。手持ちで床から大きく外れたあとに
戻れなくなるのを避けるためで、**実機の検出器には無い動き**。やり直した時刻は画面の
「出来事」に出る。

シーク (戻るコマ送りを含む) は、行き先の 0.5 秒手前 (--warmup) から取り付けを測り直して
画面に出さずに流し、行き先で表示する。追尾と姿勢はそこで作り直されるので、
続けて再生したときと少し違うことがある。
"""

import argparse
import base64
import bisect
import glob
import json
import math
import os
import re
import threading
import time
from urllib.parse import parse_qs, urlparse

import numpy as np

from . import opponent_viewer as ov
from .detect import (ATTITUDE_STALE, DetectorParams, forward_ref, Intrinsics,
                     ring_basis, RING_LOST, RingDetector, to_plane)
from .detect.geometry import Deprojector
from .rsbag import RealSenseBag

try:                                    # 無ければ color 画像を出さないだけ
    import cv2
except ImportError:                     # pragma: no cover
    cv2 = None

#: 画素の重ね描きの分類 (viz/opponent_view.html の OV_COLORS と同じ並び)。
#: 16 以上は塊の表示番号 + 16
OV_NONE, OV_VALID, OV_RING, OV_FLOOR_OTHER, OV_BELOW, OV_ABOVE, OV_OUTSIDE = range(7)
OV_CLUSTER0 = 16


# ==========================================================================
# 取り付けを測る (IMU と床の平面)
# ==========================================================================
def mount_hint_from_name(path):
    """ファイル名から俯角の見当 [deg]。'30deg' → 30、'holizontal' → 0、無ければ None。"""
    name = os.path.basename(path).lower()
    m = re.search(r'(\d+(?:\.\d+)?)deg', name)
    if m:
        return float(m.group(1))
    if 'holizontal' in name or 'horizontal' in name or 'level' in name:
        return 0.0
    return None


def up_from_pitch(pitch_deg):
    t = math.radians(pitch_deg)
    return np.array([0.0, -math.cos(t), -math.sin(t)])


def pitch_roll(u):
    """鉛直 u (カメラ座標 x右 y下 z前) から、光軸の俯角とロール [deg]。

    opponent_viewer の画面の「俯角 / ロール」と同じ式。
    """
    ux, uy, uz = (float(v) for v in u)
    return math.degrees(math.atan2(-uz, -uy)), math.degrees(math.atan2(ux, -uy))


def estimate_floor(pts, hint, max_angle_deg, near=3.0, tol=0.012, iters=400,
                   max_points=20000, seed=0):
    """近くの点から、法線が hint に近い平面を RANSAC で探す。

    返り値は None か {'normal': 上向きの単位法線, 'height': 平面の高さ (= n·p、カメラより
    下なら負), 'inliers': 採用点の割合, 'resid': 残差 [m]}。
    リングの上に立ったカメラでは、近くで一番広い水平面が自分の乗っているリング面になる。
    場外の床も同じ向きの平面だが、3 m 以内に限れば普通はリング面の方が広い。
    """
    pts = np.asarray(pts, dtype=np.float32)
    d = np.linalg.norm(pts, axis=1)
    p = pts[d < near]
    if p.shape[0] > max_points:
        p = p[:: int(math.ceil(p.shape[0] / max_points))]
    if p.shape[0] < 300:
        return None
    hint = np.asarray(hint, dtype=np.float64)
    hint = hint / np.linalg.norm(hint)
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, p.shape[0], size=(iters, 3))
    a, b, c = p[idx[:, 0]], p[idx[:, 1]], p[idx[:, 2]]
    n = np.cross(b - a, c - a).astype(np.float64)
    nn = np.linalg.norm(n, axis=1)
    ok = nn > 1e-6
    n[ok] /= nn[ok, None]
    n[np.sum(n * hint, axis=1) < 0] *= -1.0
    ok &= np.sum(n * hint, axis=1) > math.cos(math.radians(max_angle_deg))
    if not ok.any():
        return None
    n, a = n[ok], a[ok]
    off = np.sum(n * a, axis=1)
    counts = (np.abs(p.astype(np.float64) @ n.T - off[None, :]) < tol).sum(axis=0)
    best = int(np.argmax(counts))
    normal, offset = n[best], off[best]
    # 採用点で最小二乗に詰め直す (2 回)
    for _ in range(2):
        inl = np.abs(p @ normal - offset) < tol
        if inl.sum() < 100:
            return None
        q = p[inl].astype(np.float64)
        cen = q.mean(axis=0)
        _, _, vt = np.linalg.svd(q - cen, full_matrices=False)
        normal = vt[2]
        if normal @ hint < 0:
            normal = -normal
        offset = float(normal @ cen)
    inl = np.abs(p @ normal - offset) < tol
    resid = float(np.sqrt(np.mean((p[inl] @ normal - offset) ** 2)))
    if math.degrees(math.acos(min(1.0, float(normal @ hint)))) > max_angle_deg:
        return None
    return {'normal': normal, 'height': offset, 'inliers': float(inl.mean()),
            'resid': resid, 'peaks': height_peaks(pts, normal)}


def height_peaks(pts, normal, lo=-2.0, hi=-0.05, bin_w=0.01, min_share=0.2):
    """法線に沿った高さのヒストグラムの山を [(高さ, 点数), ...] で返す (高い順)。

    リング面と場外の床は平行なので、法線が決まっても高さの候補は 2 つ以上ありうる
    (会場では 32〜34 cm の段差)。一番大きい山の min_share 倍に満たない山は捨てる。
    """
    h = np.asarray(pts, dtype=np.float64) @ np.asarray(normal, dtype=np.float64)
    h = h[(h > lo) & (h < hi)]
    if h.size == 0:
        return []
    nb = int(round((hi - lo) / bin_w))
    cnt, edges = np.histogram(h, bins=nb, range=(lo, hi))
    sm = np.convolve(cnt, np.ones(3), mode='same')
    top = sm.max()
    out = []
    for k in range(nb):
        left = sm[k - 1] if k > 0 else -1
        right = sm[k + 1] if k + 1 < nb else -1
        if sm[k] >= min_share * top and sm[k] > left and sm[k] >= right:
            sel = (h >= edges[max(0, k - 2)]) & (h < edges[min(nb, k + 3)])
            out.append((float(np.median(h[sel])), int(sm[k])))
    return sorted(out, key=lambda x: -x[0])


def measure_mount(pts, imu, stamp, name_pitch, fallback_up, cam_height_fixed=None,
                  prev_height=None, keep_band=0.12):
    """1 フレームから取り付け (鉛直・俯角・ロール・カメラ高さ) を測る。

    返り値の辞書は画面の「取り付け」にそのまま出す。'up' と 'cam_height' を検出器に使う。

    カメラ高さは法線に沿った高さの山から選ぶ。最初 (prev_height が None) は
    **一番高い山** (リングは場外の床より高い)。取り直しでは前の高さから keep_band
    以内の山を採り、無ければ前の高さのまま (リング面が写っていないフレームで場外の床を
    リング面と取り違えないため。窓が合っていれば検出器がリング面を拾い直す)。
    """
    info = {'source': '', 'imu_plane_deg': None, 'accel_sd': None, 'inliers': None,
            'resid_mm': None}
    acc = None
    if imu is not None and not imu.empty:
        acc, sd, n = imu.accel_around(stamp, 0.25)
        if acc is not None:
            info['accel_sd'] = round(sd, 2)
    if acc is not None and np.linalg.norm(acc) > 1.0:
        hint = acc / np.linalg.norm(acc)
        max_angle = 15.0
        src = 'IMU'
    elif name_pitch is not None:
        hint = up_from_pitch(name_pitch)
        max_angle = 40.0
        src = 'ファイル名 %.0f°' % name_pitch
    else:
        hint = np.asarray(fallback_up, dtype=np.float64)
        max_angle = 40.0
        src = '直前の鉛直'
    floor = estimate_floor(pts, hint, max_angle)
    if floor is not None:
        up = floor['normal']
        info['source'] = src + ' → 床の平面'
        info['inliers'] = round(floor['inliers'], 2)
        info['resid_mm'] = round(floor['resid'] * 1e3, 1)
        peaks = floor['peaks']
        cam_h = -floor['height']
        if prev_height is not None:
            near = [pk for pk in peaks if abs(-pk[0] - prev_height) <= keep_band]
            if near:
                cam_h = -min(near, key=lambda pk: abs(-pk[0] - prev_height))[0]
            else:
                cam_h = prev_height
                info['source'] += ' (高さは前のまま)'
        elif peaks:
            cam_h = -peaks[0][0]
        info['peaks'] = [[round(-pk[0], 3), pk[1]] for pk in peaks[:4]]
        if acc is not None:
            info['imu_plane_deg'] = round(math.degrees(math.acos(
                min(1.0, float(up @ (acc / np.linalg.norm(acc)))))), 2)
    else:
        up = hint
        info['source'] = src + ' のみ (床の平面が取れない)'
        cam_h = None
    if cam_height_fixed is not None:
        cam_h = cam_height_fixed
    pitch, roll = pitch_roll(up)
    info.update({'up': up, 'cam_height': cam_h, 'pitch_deg': round(pitch, 1),
                 'roll_deg': round(roll, 1)})
    return info


# ==========================================================================
# 画面の状態 (opponent_viewer.ViewState に bag の分を足す)
# ==========================================================================
def _b64(a):
    return base64.b64encode(np.ascontiguousarray(a).tobytes()).decode('ascii')


class BagViewState(ov.ViewState):
    """ViewState に、再生位置・塊の番号・画素ごとの塊・側面図を足したもの。"""

    depth_period = 0.1

    def __init__(self, params, source, view_hz=10.0):
        super().__init__(params, source, view_hz)
        self.bag_t = 0.0
        self.bag_info = {}
        self.spec = None
        self._res = None
        self._color = None
        self._cache_key = None
        self._cache = None

    # 出来事の時刻は bag の中の時刻にする
    def _event(self, text):
        self.events.append({'t': round(self.bag_t, 1), 'text': text})
        del self.events[:-40]

    def note(self, text):
        with self.lock:
            self._event(text)

    def reset_fall(self):
        with self.lock:
            if self.fall is not None:
                self.fall.reset()
            self.prev_fallen = None
            self.prev_status = None

    def feed_frame(self, res, depth, scale, dt, intr, bag_t, color=None, force=False):
        with self.lock:
            self._res = res
            self.bag_t = bag_t
            if color is not None:
                self._color = color
            if force:
                self.last_pack = 0.0
                self.last_depth_pack = 0.0
        self.feed(res, depth, scale, dt, intr)

    def set_bag_info(self, info):
        with self.lock:
            self.bag_info = info
            if isinstance(self.state, dict) and self.state.get('ready'):
                self.state = dict(self.state, bag=info)

    # ------------------------------------------------------------ 点ごとの量
    def _points(self, res):
        """点ごとの (fwd, left, 面からの高さ, 塊の表示番号) と、表示番号の表。

        表示番号は塊を近い順に並べた 1, 2, ...。画面の表の行と同じ並び。
        """
        if self._cache_key is res:
            return self._cache
        order = sorted(res.clusters, key=lambda c: c.radius)
        n_lab = max([c.label for c in res.clusters] + [0]) + 1
        lut = np.zeros(n_lab, dtype=np.int32)
        for k, c in enumerate(order):
            lut[c.label] = k + 1
        pts = None
        if (res.points is not None and res.up is not None
                and res.ring_height is not None):
            e1, e2 = ring_basis(res.up, forward_ref(self.p.body.cam_pitch_deg))
            h, fwd, left = to_plane(res.points, res.up, e1, e2)
            rel = h - res.ring_height
            ids = np.zeros(rel.shape[0], dtype=np.int32)
            if res.point_labels is not None and res.point_labels.shape[0] == rel.shape[0]:
                lab = res.point_labels
                ids = lut[np.minimum(lab, n_lab - 1)] * (lab < n_lab)
            pts = (fwd, left, rel, ids)
        self._cache_key, self._cache = res, (pts, lut, order)
        return self._cache

    # ------------------------------------------------------------ 詰め替え
    def _pack(self, res, opp, fallen, fresh, truth):
        out = super()._pack(res, opp, fallen, fresh, truth)
        pts, lut, order = self._points(res)
        for k, c in enumerate(out['clusters']):
            c['id'] = k + 1
            src = order[k]
            c['voxel_bottom'] = ov._num(src.height_min)
            c['covered'] = bool(src.covered)
            c['overhead_cells'] = int(src.overhead_cells)
        if out.get('grid') is not None and res.obj_labels is not None:
            lab = res.obj_labels
            ids = np.where(lab < lut.size, lut[np.minimum(lab, lut.size - 1)], 0)
            out['grid']['ids'] = _b64(np.clip(ids, 0, 255).astype(np.uint8))
        out['side'] = self._side(res, pts)
        out['bag'] = self.bag_info
        out['bag_t'] = round(self.bag_t, 2)
        return out

    def _side(self, res, pts, max_points=6000):
        """側面図の点。面より上の点を、塊の番号 (塊でなければ 0) と一緒に送る。"""
        if pts is None:
            return None
        fwd, left, rel, ids = pts
        t = self.p.tune
        m = (rel > t.obj_h_lo) & (rel < t.obj_h_hi) & (np.abs(fwd) < 6) & (np.abs(left) < 6)
        idx = np.flatnonzero(m)
        if idx.size > max_points:
            # 塊の点を優先して残す
            inc = idx[ids[idx] > 0]
            rest = idx[ids[idx] == 0]
            if inc.size > max_points:
                inc = inc[:: int(math.ceil(inc.size / max_points))]
            room = max(0, max_points - inc.size)
            if rest.size > room and room > 0:
                rest = rest[:: int(math.ceil(rest.size / room))]
            elif room == 0:
                rest = rest[:0]
            idx = np.concatenate([inc, rest])

        def mm(v):
            return _b64(np.clip(np.round(v[idx] * 1000.0), -32000, 32000).astype('<i2'))
        return {'n': int(idx.size), 'fwd': mm(fwd), 'left': mm(left), 'h': mm(rel),
                'id': _b64(np.clip(ids[idx], 0, 255).astype(np.uint8))}

    def _pack_depth(self, depth, scale):
        res = self._res
        t = self.p.tune
        s = int(t.stride)
        z = depth[::s, ::s].astype(np.float32) * np.float32(scale)
        ok = (z > t.depth_min) & (z < t.depth_max)
        z_near, z_far = 0.15, 4.0
        v = np.clip((z - z_near) / (z_far - z_near), 0.0, 1.0)
        img = np.where(z > 0, 1 + np.round(v * 254), 0).astype(np.uint8)
        code = np.zeros(z.shape, dtype=np.uint8)
        code[ok] = OV_VALID
        pts, _, order = self._points(res) if res is not None else (None, None, [])
        if pts is not None and pts[0].shape[0] == int(ok.sum()) and self.spec is not None:
            fwd, left, rel, ids = pts
            c = np.full(rel.shape[0], OV_VALID, dtype=np.uint8)
            iu, iv, inside = self.spec.index(fwd, left)
            iu = np.clip(iu, 0, self.spec.nu - 1)
            iv = np.clip(iv, 0, self.spec.nv - 1)
            floor = np.abs(rel) < t.floor_band
            c[floor] = OV_FLOOR_OTHER
            if res.ring_mask is not None:
                c[floor & inside & res.ring_mask[iu, iv]] = OV_RING
            c[rel < -t.below_band] = OV_BELOW
            above = (rel > t.obj_h_lo) & (rel < t.obj_h_hi)
            c[above] = OV_ABOVE
            if res.outside_mask is not None:
                c[above & inside & res.outside_mask[iu, iv]] = OV_OUTSIDE
            inc = ids > 0
            c[inc] = np.clip(OV_CLUSTER0 + ids[inc], 0, 255).astype(np.uint8)
            code[ok] = c
        out = {'ready': True, 'w': int(img.shape[1]), 'h': int(img.shape[0]),
               'near': z_near, 'far': z_far, 'data': _b64(img), 'ov': _b64(code),
               # 画素の塊の番号 → 種別。状態 (/state) とは別に取りに来るので、ここで閉じる
               'clusters': [{'id': k + 1, 'kind': c.kind,
                             'selected': res is not None and c is res.selected}
                            for k, c in enumerate(order)]}
        if self._color is not None and cv2 is not None:
            okj, jpg = cv2.imencode('.jpg', self._color[:, :, ::-1],
                                    [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if okj:
                out['color'] = 'data:image/jpeg;base64,' + base64.b64encode(
                    jpg.tobytes()).decode('ascii')
            self._color = None
        return out


# ==========================================================================
# 再生
# ==========================================================================
def load_flat(config):
    """opponent_detector.yaml を平らな辞書で読む (height_probe と同じ読み方)。"""
    if not config:
        return {}
    from .height_probe import flat_from_yaml
    return flat_from_yaml(config)


def default_config():
    try:
        from .height_probe import default_config as dc
        return dc()
    except Exception:                                           # noqa: BLE001
        here = os.path.dirname(os.path.abspath(__file__))
        p = os.path.join(here, '..', 'config', 'opponent_detector.yaml')
        return p if os.path.exists(p) else ''


class BagPlayer(threading.Thread):
    """bag を 1 フレームずつ検出器に通して、画面の状態へ流す。"""

    SPEEDS = (0.1, 0.25, 0.5, 1.0, 2.0, 0.0)          # 0 = 待たずに流す

    def __init__(self, view, paths, base_flat, args):
        super().__init__(daemon=True)
        self.view = view
        self.paths = paths
        self.base_flat = dict(base_flat)
        self.args = args
        self.cv = threading.Condition()
        self.playing = not args.paused
        self.speed = float(args.speed)
        self.loop = bool(args.loop)
        self.reacquire = not args.no_reacquire
        self._stop = False
        self._cmd = []                  # ('open', k) / ('seek', i) / ('step', n) / ('reinit',)
        self.k = -1
        self.rb = None
        self.det = None
        self.i = 0                      # 次に流すフレーム
        self.shown = -1                 # 画面に出ているフレーム
        self.bad_run = 0
        self.mount = None
        self.n_reacquire = 0
        self.t_prev = None
        self.anchor = None              # (壁時計, bag の時刻) 再生の速さの基準
        self.last_color = 0.0
        self.proc_ms = []
        self._open(max(0, min(len(paths) - 1, args.index)), start=args.start)

    # ------------------------------------------------------------ 操作 (HTTP から)
    def command(self, cmd, **kw):
        with self.cv:
            if cmd == 'play':
                self.playing = True
                self.anchor = None
            elif cmd == 'pause':
                self.playing = False
            elif cmd == 'toggle':
                self.playing = not self.playing
                self.anchor = None
            elif cmd == 'speed':
                self.speed = float(kw.get('v', 1.0))
                self.anchor = None
            elif cmd == 'loop':
                self.loop = kw.get('v', '1') not in ('0', 'false')
            elif cmd == 'reacquire':
                self.reacquire = kw.get('v', '1') not in ('0', 'false')
            elif cmd == 'step':
                self.playing = False
                self._cmd.append(('step', int(kw.get('n', 1))))
            elif cmd == 'seek':
                self._cmd.append(('seek', int(kw.get('i', 0))))
            elif cmd == 'seek_t':
                self._cmd.append(('seek_t', float(kw.get('t', 0.0))))
            elif cmd == 'open':
                self._cmd.append(('open', int(kw.get('k', 0))))
            elif cmd == 'reinit':
                self._cmd.append(('reinit',))
            else:
                return False
            self.cv.notify()
        self._publish_info()
        return True

    def stop(self):
        with self.cv:
            self._stop = True
            self.cv.notify()

    # ------------------------------------------------------------ bag と検出器
    def _open(self, k, start=0.0):
        if self.rb is not None:
            self.rb.close()
        path = self.paths[k]
        t0 = time.monotonic()
        self.rb = RealSenseBag(path)
        rb = self.rb
        w, h = rb.intr_wh
        kk = rb.K
        self.intr = Intrinsics(w, h, kk[0], kk[4], kk[2], kk[5])
        self.k = k
        self.name_pitch = (self.args.pitch if self.args.pitch is not None
                           else mount_hint_from_name(path))
        self.t0_bag = rb.depth_index[0][0] if len(rb) else 0.0
        self.view.source = 'bag: ' + os.path.basename(path)
        self.view.reset_fall()
        self.view.note('開いた: %s (%d 枚 %.1f 秒, 読み込み %.1f 秒%s%s)' % (
            os.path.basename(path), len(rb), rb.duration, time.monotonic() - t0,
            '' if rb.imu.empty else ', IMU あり',
            '' if rb.bag.indexed else ', 索引なし (末尾が切れている)'))
        print('[bag] %s: %d 枚 %.1f 秒 depth %dx%d IMU %s%s' % (
            os.path.basename(path), len(rb), rb.duration, w, h,
            'なし' if rb.imu.empty else 'gyro %d / accel %d' % (
                rb.imu.gyro_t.size, rb.imu.accel_t.size),
            '' if rb.bag.indexed else ' (索引なし・末尾が切れている)'), flush=True)
        self._seek(self._index_at(start))

    def _index_at(self, t):
        rb = self.rb
        target = self.t0_bag + float(t)
        ts = [m[0] for m in rb.depth_index]
        return max(0, min(len(ts) - 1, bisect.bisect_left(ts, target)))

    def _points(self, depth):
        t = self.base_params().tune
        dp = Deprojector(self.intr, t.stride, t.border_px)
        pts, _ = dp(depth, self.rb.depth_scale, t.depth_min, t.depth_max)
        return pts

    def base_params(self):
        return DetectorParams.from_flat(self.base_flat)

    def _init_at(self, i, why):
        """フレーム i で取り付けを測り、検出器を作り直す。"""
        stamp, depth = self.rb.depth(i)
        base = self.base_params()
        fallback = (self.det.attitude.u if self.det is not None
                    else base.body.up_from_mount)
        prev_h = (self.det.p.body.cam_height
                  if self.det is not None and why not in ('開始', 'シーク', '手動') else None)
        m = measure_mount(self._points(depth), self.rb.imu, stamp, self.name_pitch,
                          fallback, self.args.cam_height, prev_height=prev_h)
        flat = dict(self.base_flat)
        flat['body.cam_pitch_deg'] = float(m['pitch_deg'])
        if m['cam_height'] is not None:
            flat['body.cam_height'] = float(m['cam_height'])
        params = DetectorParams.from_flat(flat)
        self.det = RingDetector(params)
        self.det.attitude.reset(up=m['up'])
        self.view.p = params
        self.view.spec = self.det.spec
        self.mount = dict(m, up=[round(float(x), 4) for x in m['up']],
                          cam_height=(round(m['cam_height'], 3)
                                      if m['cam_height'] is not None else None),
                          why=why, at=round(self.rb.record_time(i) - self.t0_bag, 2),
                          cam_height_used=round(params.body.cam_height, 3))
        self.bad_run = 0
        self.t_prev = None
        extra = ''
        if m['imu_plane_deg'] is not None:
            extra = ' / IMU と床の法線の差 %.1f°' % m['imu_plane_deg']
        if m.get('peaks'):
            extra += ' / 高さの山 ' + ', '.join('%.3f' % pk[0] for pk in m['peaks'])
        self.view.note('取り付けを測った (%s): %s / 俯角 %.1f° ロール %.1f° / 高さ %s%s' % (
            why, m['source'], m['pitch_deg'], m['roll_deg'],
            ('%.3f m' % m['cam_height']) if m['cam_height'] is not None
            else '設定のまま %.3f m' % params.body.cam_height, extra))

    def _seek(self, i, why=None):
        """フレーム i を画面に出す。warmup 枚だけ手前から、画面に出さずに流す。"""
        n = len(self.rb)
        i = max(0, min(n - 1, int(i)))
        start = max(0, i - int(self.args.warmup))
        self._init_at(start, why or ('シーク' if self.shown >= 0 else '開始'))
        self.view.reset_fall()
        for j in range(start, i):
            self._step_frame(j, show=False)
        self.i = i
        self._step_frame(i, show=True, force=True)
        self.i = i + 1
        self.anchor = None

    # ------------------------------------------------------------ 1 フレーム
    def _step_frame(self, i, show=True, force=False):
        stamp, depth = self.rb.depth(i)
        dt = (stamp - self.t_prev) if self.t_prev is not None else 1.0 / 30.0
        dt = min(max(dt, 1e-3), 0.5)
        gyro = self.rb.imu.gyro_between(self.t_prev, stamp) if self.t_prev is not None else []
        t0 = time.perf_counter()
        res = self.det.step(depth, self.intr, dt, gyro=gyro,
                            depth_scale=self.rb.depth_scale, want_debug=show)
        self.proc_ms.append((time.perf_counter() - t0) * 1e3)
        del self.proc_ms[:-60]
        self.t_prev = stamp
        bad = res.status in (RING_LOST, ATTITUDE_STALE)
        self.bad_run = self.bad_run + 1 if bad else 0
        if not show:
            return res
        self.shown = i
        now = time.monotonic()
        color = None
        if force or now - self.last_color >= 0.2:
            color = self.rb.color_near(i)
            self.last_color = now
        self.view.feed_frame(res, depth, self.rb.depth_scale, dt, self.intr,
                             self.rb.record_time(i) - self.t0_bag, color=color, force=force)
        self._publish_info()
        if self.reacquire and self.bad_run >= int(self.args.reacquire_frames):
            self.n_reacquire += 1
            self._init_at(min(i + 1, len(self.rb) - 1),
                          '%s が %d 枚続いた' % (res.status, self.bad_run))
        return res

    def _publish_info(self):
        try:
            self.view.set_bag_info(self._info())
        except (IndexError, AttributeError):   # bag を開き直している途中
            pass

    def _info(self):
        rb = self.rb
        return {
            'files': [os.path.basename(p) for p in self.paths], 'k': self.k,
            'frame': self.shown, 'n': len(rb),
            't': round(rb.depth_index[max(0, self.shown)][0] - self.t0_bag, 2) if len(rb) else 0,
            'duration': round(rb.duration, 2),
            'playing': self.playing, 'speed': self.speed, 'loop': self.loop,
            'reacquire': self.reacquire, 'n_reacquire': self.n_reacquire,
            'imu': not rb.imu.empty, 'indexed': rb.bag.indexed,
            'depth_wh': list(rb.intr_wh), 'color': bool(rb.color_index),
            'mount': self.mount,
            'proc_ms': round(float(np.mean(self.proc_ms)), 1) if self.proc_ms else None,
        }

    # ------------------------------------------------------------ 回す
    def run(self):
        while True:
            with self.cv:
                while not self._stop and not self._cmd and not self.playing:
                    self.cv.wait(0.5)
                if self._stop:
                    return
                cmd = self._cmd.pop(0) if self._cmd else None
            try:
                if cmd is not None:
                    self._do(cmd)
                elif self.i >= len(self.rb):
                    if self.loop:
                        self._seek(0)
                    else:
                        with self.cv:
                            self.playing = False
                        self._publish_info()
                else:
                    self._pace(self.i)
                    self._step_frame(self.i, show=True)
                    self.i += 1
            except Exception as e:                              # noqa: BLE001
                self.view.note('エラー: %s' % e)
                print('[bag] エラー: %r' % e, flush=True)
                with self.cv:
                    self.playing = False

    def _do(self, cmd):
        if cmd[0] == 'open':
            self.shown = -1
            self._open(max(0, min(len(self.paths) - 1, cmd[1])))
        elif cmd[0] == 'seek':
            self._seek(cmd[1])
        elif cmd[0] == 'seek_t':
            self._seek(self._index_at(cmd[1]))
        elif cmd[0] == 'step':
            n = cmd[1]
            if n == 1 and self.i < len(self.rb):
                self._step_frame(self.i, show=True, force=True)
                self.i += 1
            else:
                self._seek(self.shown + n)
        elif cmd[0] == 'reinit':
            self._seek(max(0, self.shown), why='手動')

    def _pace(self, i):
        if self.speed <= 0:
            return
        t_bag = self.rb.depth_index[i][0]
        now = time.monotonic()
        if self.anchor is None:
            self.anchor = (now, t_bag)
            return
        wait = self.anchor[0] + (t_bag - self.anchor[1]) / self.speed - now
        if wait > 0.5:                      # 大きく飛んだら基準を置き直す
            self.anchor = (now, t_bag)
        elif wait > 0:
            with self.cv:
                self.cv.wait(wait)
        elif wait < -0.5:                   # 処理が追いつかない。遅れを溜めない
            self.anchor = (now, t_bag)


# ==========================================================================
# HTTP (opponent_viewer の Handler に /bag を足す)
# ==========================================================================
PLAYER = None


class BagHandler(ov.Handler):

    def do_POST(self):
        u = urlparse(self.path)
        if u.path == '/bag':
            q = {k: v[-1] for k, v in parse_qs(u.query).items()}
            cmd = q.pop('cmd', '')
            self._json({'ok': int(PLAYER.command(cmd, **q))})
            return
        if u.path == '/reset_attitude':           # 「基準姿勢を取り直す」= 取り付けを測り直す
            PLAYER.command('reinit')
            self._json({'ok': 1})
            return
        super().do_POST()


# ==========================================================================
# 画面なしで流す (--batch)
# ==========================================================================
def run_batch(paths, base_flat, args):
    """最後まで流して、1 秒ごとに状態・塊・選んだ相手を出す。"""
    class _Null:
        p = None

        def note(self, text):
            print('   ', text)

        def reset_fall(self):
            pass

        def set_bag_info(self, info):
            pass

        def feed_frame(self, *a, **k):
            pass

    for k in range(len(paths)):
        null = _Null()
        a = argparse.Namespace(**vars(args))
        a.index, a.paused = k, True
        pl = BagPlayer(null, paths, base_flat, a)
        rb = pl.rb
        print('=' * 72)
        print(os.path.basename(paths[k]))
        print('  t[s]  状態            塊 相手(距離 方位 上端 種別)          鉛直との差 高さ実測 処理ms')
        counts = {}
        last_sec = -1
        ms = []
        for i in range(pl.i, len(rb)):
            t0 = time.perf_counter()
            res = pl._step_frame(i, show=False)
            ms.append((time.perf_counter() - t0) * 1e3)
            counts[res.status] = counts.get(res.status, 0) + 1
            if (pl.reacquire and pl.bad_run >= int(args.reacquire_frames)
                    and i + 1 < len(rb)):
                pl.n_reacquire += 1
                pl._init_at(i + 1, '%s が %d 枚続いた' % (res.status, pl.bad_run))
            tb = rb.depth_index[i][0] - pl.t0_bag
            if int(tb) != last_sec:
                last_sec = int(tb)
                sel = res.selected
                opp = '-'
                if sel is not None:
                    opp = '%.2fm %+4.0f° %.2fm %s' % (
                        sel.radius, math.degrees(sel.bearing), sel.top_height, sel.kind)
                print('  %5.1f %-15s %2d %-34s %5.1f° %6s %6.1f' % (
                    tb, res.status, len(res.clusters), opp, res.plane_angle_deg,
                    '%.3f' % -res.ring_height if res.ring_height is not None else '-',
                    ms[-1]))
        tot = sum(counts.values())
        share = ['%s %.0f%%' % (s, 100.0 * n / tot) for s, n in sorted(counts.items())]
        print('  状態の割合: ' + ', '.join(share))
        print('  処理 mean %.1f ms / p95 %.1f ms / 取り直し %d 回' % (
            np.mean(ms), np.percentile(ms, 95), pl.n_reacquire))
        rb.close()
    return 0


# ==========================================================================
def _collect(inputs):
    paths = []
    for p in inputs:
        if os.path.isdir(p):
            paths += sorted(glob.glob(os.path.join(p, '*.bag')))
        elif os.path.exists(p):
            paths.append(p)
        else:
            raise SystemExit('見つからない: %s' % p)
    if not paths:
        raise SystemExit('.bag が無い: %s' % ' '.join(inputs))
    return paths


def main(argv=None):
    global PLAYER
    ap = argparse.ArgumentParser(
        description='RealSense の録画 (.bag) を検出器に通して、塊の状態をブラウザで見る')
    ap.add_argument('bags', nargs='+', help='.bag か、.bag の入ったフォルダ')
    ap.add_argument('--port', type=int, default=8106)
    ap.add_argument('--bind', default='0.0.0.0')
    ap.add_argument('--config', default=None,
                    help='opponent_detector.yaml (既定は入っているもの)')
    ap.add_argument('--index', type=int, default=0, help='最初に開く bag の番号')
    ap.add_argument('--start', type=float, default=0.0, help='開始位置 [s]')
    ap.add_argument('--speed', type=float, default=1.0, help='再生の速さ (0 で待たない)')
    ap.add_argument('--paused', action='store_true', help='止めた状態で開く')
    ap.add_argument('--loop', action='store_true', help='最後まで行ったら頭から')
    ap.add_argument('--pitch', type=float, default=None,
                    help='IMU の無い bag の俯角の見当 [deg] (既定はファイル名から)')
    ap.add_argument('--cam-height', type=float, default=None,
                    help='カメラ高さを測らずにこの値にする [m]')
    ap.add_argument('--no-reacquire', action='store_true',
                    help='床を見失っても取り付けを測り直さない (実機と同じ)')
    ap.add_argument('--reacquire-frames', type=int, default=15,
                    help='RING_LOST / ATTITUDE_STALE が何枚続いたら測り直すか')
    ap.add_argument('--warmup', type=int, default=15,
                    help='シークのとき、行き先の何枚手前から流すか')
    ap.add_argument('--view-hz', type=float, default=10.0)
    ap.add_argument('--batch', action='store_true',
                    help='画面を出さずに最後まで流して、1 秒ごとの要約を出す')
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                    help='パラメータを上書きする (例: --set tune.cell_z=0.05)。何度でも')
    args = ap.parse_args(argv)

    paths = _collect(args.bags)
    config = args.config if args.config is not None else default_config()
    base_flat = load_flat(config)
    for kv in args.set:
        k, _, v = kv.partition('=')
        try:
            base_flat[k] = json.loads(v)
        except ValueError:
            base_flat[k] = v
    extra = (' + ' + ' '.join(args.set)) if args.set else ''
    print('設定: %s%s' % (config or '(params.py の既定)', extra))

    if args.batch:
        return run_batch(paths, base_flat, args)

    params = DetectorParams.from_flat(base_flat)
    view = BagViewState(params, 'bag', args.view_hz)
    ov.VIEW = view
    PLAYER = BagPlayer(view, paths, base_flat, args)
    view.source = 'bag: ' + os.path.basename(paths[PLAYER.k])
    PLAYER.start()

    srv = ov.ThreadingHTTPServer((args.bind, args.port), BagHandler)
    print('=' * 72)
    print('bag を検出器に通して見る (実機にもトピックにも触れない)')
    for p in paths:
        print('   %s' % p)
    if args.bind == '0.0.0.0':
        for name, addr in ov._local_addrs():
            print('   %-6s http://%s:%d/' % (name, addr, args.port))
    else:
        print('   http://%s:%d/' % (args.bind, args.port))
    print('   停止は Ctrl-C')
    print('=' * 72, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\n停止')
    finally:
        PLAYER.stop()
        srv.server_close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
