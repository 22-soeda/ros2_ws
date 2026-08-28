# -*- coding: utf-8 -*-
"""検出パイプラインの本体。depth 1 枚から相手とリングの縁までを出す。

docs/opponent_detection.pdf §3 の段の並びをそのまま実装したもの。

    間引きと逆投影 → 姿勢 (ジャイロ予測) → 高さヒストグラムで h_r
        → 近距離の面あてはめ → 法線で u を補正 (ここが閉ループ)
        → 占有グリッド → リング連結成分 → 境界セル → d_cliff
                                       → 面上の点をクラスタリング → α-β 追尾

要点は 2 つある。

第一に、**姿勢推定が知覚の前段ではなく、知覚と閉ループを組んでいる。** リング面は
水平な板なので、その法線はそのまま鉛直の観測値になる。IMU はフレーム間をつなぐ
ためだけに使い、絶対の傾きは毎フレーム depth から取り直す。姿勢と知覚を独立 2 段に
せず 1 つのループに閉じる (§5.4)。

第二に、**リング面の連結成分を先に確定させ、以降の探索をその内側に閉じている。**
物体クラスタリングとエッジ抽出は同じ占有グリッドから枝分かれするだけで、独立の
アルゴリズムを持たない (§7, §8.1)。

ROS には依存しない。入力は numpy の深度画像と内部パラメータと IMU サンプルだけで、
同じ入力からは同じ出力が出る。単体テストは合成シーンを描いてここへ通す
(roboone_motion の walk_core と同じ流儀)。
"""

from dataclasses import dataclass, field
import time

import numpy as np

from . import clusters as cl
from . import edge as ed
from . import grid as g
from . import ring as rg
from .attitude import AttitudeEstimator
from .geometry import Deprojector, forward_ref, ring_basis, to_plane
from .params import DetectorParams
from .tracker import AlphaBetaTracker

# 状態 (§9.2)。「相手なし」を 1 つに潰さないための区別
OK = 'OK'
NO_OPPONENT = 'NO_OPPONENT'          # 正常だが条件を通るクラスタがない
ATTITUDE_STALE = 'ATTITUDE_STALE'    # 面あてはめの門を通らず、姿勢がジャイロ任せ
RING_LOST = 'RING_LOST'              # リング面自体が取れない


@dataclass
class DetectionResult:
    """1 フレームの結果。デバッグ表示に要るものまで全部入れてある。"""

    status: str = RING_LOST
    up: np.ndarray = None              # 鉛直 u (カメラ座標)
    ring_height: float = None          # h_r [m] カメラ原点からリング面まで (負)
    ring_area: float = 0.0             # [m^2] ±30mm のスライスに残った面積
    plane_resid: float = float('nan')  # [m] 面あてはめの残差
    plane_corrected: bool = False      # このフレームで u が引き戻されたか
    n_points: int = 0

    # 相手 (追尾後)。status が OK でなくても、取れているときは埋める
    position: tuple = None             # (前方 u, 左 v) [m] リング平面座標
    velocity: tuple = (0.0, 0.0)       # [m/s]
    height: float = float('nan')       # [m] 重心のリング面からの高さ
    top_height: float = float('nan')   # [m] 上端 z_top
    width: float = float('nan')        # [m] max(w, d)
    extrapolated: bool = False

    cliff: np.ndarray = None           # d_cliff(θ)。見えていない方位は NaN

    # --- 以下はデバッグ・テスト用 ----------------------------------------
    clusters: list = field(default_factory=list)
    selected: object = None
    ring_mask: np.ndarray = None
    obj_mask: np.ndarray = None
    above_mask: np.ndarray = None      # 面より上の全セル (リング外も含む)
    fov_cells: np.ndarray = None
    spec: object = None
    timings: dict = field(default_factory=dict)

    @property
    def valid(self):
        return self.status == OK and self.position is not None


class RingDetector:
    """フレームをまたぐ状態 (姿勢・追尾) を持つ検出器。"""

    def __init__(self, params: DetectorParams = None):
        self.p = params or DetectorParams()
        t = self.p.tune
        self.attitude = AttitudeEstimator(
            self.p.body.up_from_mount, blend=t.plane_blend,
            resid_max=t.fit_resid_max, angle_max_deg=t.fit_angle_max_deg,
            stale_frames=t.stale_frames)
        self.tracker = AlphaBetaTracker(
            alpha=t.track_alpha, beta=t.track_beta,
            gate=t.track_gate, max_coast=t.track_max_coast)
        self.spec = g.GridSpec(cell=t.cell,
                               u_min=-t.grid_back, u_max=t.grid_forward,
                               v_min=-t.grid_side, v_max=t.grid_side)
        self._deproj = None
        self._seed = None
        #: 方位の基準にする機体前方 (カメラ座標)。光軸ではない (geometry の注記)
        self._fwd_ref = forward_ref(self.p.body.cam_pitch_deg)
        #: 追尾が外挿している間は、最後に観測した寸法を保つ
        self._last_shape = (float('nan'), float('nan'), float('nan'))
        self.frames = 0

    # ---------------------------------------------------------------- 種
    def _seed_mask(self):
        """リング成分の種にするセル (params.TuneParams.seed_* の注記を参照)。"""
        if self._seed is None:
            t = self.p.tune
            iu = np.arange(self.spec.nu)[:, None]
            iv = np.arange(self.spec.nv)[None, :]
            fwd, left = self.spec.centers(iu, iv)
            self._seed = ((fwd >= t.seed_near) & (fwd <= t.seed_far)
                          & (np.abs(left) <= t.seed_half_width))
        return self._seed

    def _deprojector(self, intr):
        t = self.p.tune
        if self._deproj is None or not self._deproj.matches(intr, t.stride,
                                                            t.border_px):
            self._deproj = Deprojector(intr, t.stride, t.border_px)
        return self._deproj

    # ---------------------------------------------------------------- 本体
    def step(self, depth, intr, dt, gyro=(), accel=None, depth_scale=0.001,
             want_debug=False):
        """深度画像 1 枚を処理する。

        引数:
            depth        uint16 の深度画像 (mm)。depth_scale で m に直す
            intr         geometry.Intrinsics
            dt           前フレームからの経過時間 [s]
            gyro         [(omega[3], dt), ...] 前フレームからのジャイロ。200Hz 全部
            accel        起動直後に u を置くための加速度。以降は使わない
            want_debug   俯瞰表示のための中間結果も残すか (現状は計算量に差はない)
        """
        t = self.p.tune
        res = DetectionResult()
        clk = {}
        t0 = time.perf_counter()

        # --- 逆投影 (§4) -----------------------------------------------
        pts, border = self._deprojector(intr)(depth, depth_scale,
                                              t.depth_min, t.depth_max)
        res.n_points = int(pts.shape[0])
        clk['deproject'] = time.perf_counter() - t0

        # --- 姿勢: ジャイロで運ぶ (§5.4 予測) ---------------------------
        t1 = time.perf_counter()
        if self.frames == 0 and accel is not None:
            self.attitude.init_from_accel(accel)
        self.attitude.predict_samples(gyro)
        clk['attitude'] = time.perf_counter() - t1
        self.frames += 1

        if res.n_points == 0:
            self.attitude.missed()
            return self._finish(res, dt, clk)

        # --- リング面の高さ (§6) ----------------------------------------
        t2 = time.perf_counter()
        u = self.attitude.u
        e1, e2 = ring_basis(u, self._fwd_ref)
        h, fwd, left = to_plane(pts, u, e1, e2)
        h_r, n_win = rg.ring_height(h, self.p.body.cam_height, t.hist_window,
                                    t.hist_bin, t.hist_refine, t.hist_min_points)
        if h_r is None:
            self.attitude.missed()
            res.up = u
            clk['plane'] = time.perf_counter() - t2
            return self._finish(res, dt, clk)

        # --- 面あてはめ → u を引き戻す (§5.4 補正。ここが閉ループ) ------
        fit = rg.fit_plane(pts, h, fwd, left, h_r, t.fit_band, t.fit_radius,
                           t.fit_min_points, t.fit_max_points)
        if fit.ok:
            res.plane_resid = fit.resid
            before = self.attitude.u.copy()
            res.plane_corrected = self.attitude.correct(fit.normal, fit.resid)
            # 引き戻しが実際に効いた (0.02° 以上回った) ときだけ測り直す。
            # 定常状態では u と法線が既に一致していて、測り直しても値が変わらない
            moved = res.plane_corrected and float(
                np.linalg.norm(self.attitude.u - before)) > 3.5e-4
            if moved:
                # 補正した鉛直で測り直す。ここを省くと、その場で使う高さは
                # 1 フレーム古い姿勢のままになり、閉ループが 1 周遅れる
                u = self.attitude.u
                e1, e2 = ring_basis(u, self._fwd_ref)
                h, fwd, left = to_plane(pts, u, e1, e2)
                h_r, n_win = rg.ring_height(h, self.p.body.cam_height,
                                            t.hist_window, t.hist_bin,
                                            t.hist_refine, t.hist_min_points)
                if h_r is None:
                    res.up = u
                    clk['plane'] = time.perf_counter() - t2
                    return self._finish(res, dt, clk)
        else:
            self.attitude.missed()
        res.up = u
        res.ring_height = h_r
        clk['plane'] = time.perf_counter() - t2

        # --- 占有グリッドとリング連結成分 (§7) --------------------------
        t3 = time.perf_counter()
        floor = np.abs(h - h_r) < t.floor_band
        occ = g.count_cells(self.spec, fwd[floor], left[floor]) > 0
        occ = g.close(occ, 1)
        labels, n = g.label_components(occ)
        pick = g.component_of_seed(labels, n, self._seed_mask(),
                                   t.seed_fallback_to_largest)
        ring_mask = (labels == pick) if pick else np.zeros(self.spec.shape, bool)
        res.ring_mask = ring_mask
        res.ring_area = float(np.count_nonzero(ring_mask)) * t.cell * t.cell
        # 視野の縁に接するセル。境界がここに乗る方位は d_cliff を NaN にする
        fov = g.any_cells(self.spec, fwd[floor], left[floor], border[floor])
        res.fov_cells = fov
        clk['grid'] = time.perf_counter() - t3

        if not ring_mask.any():
            return self._finish(res, dt, clk)

        # --- リングの縁 (§7) --------------------------------------------
        # 面より上のセルは、エッジ側では「影を作るもの」として先に要る。
        # (物体クラスタリングはこの後、リング成分の内側に閉じてから改めて行う)
        t4 = time.perf_counter()
        above = (h - h_r > t.obj_h_lo) & (h - h_r < t.obj_h_hi)
        res.above_mask = g.count_cells(self.spec, fwd[above], left[above]) > 0
        res.cliff = ed.cliff_distances(self.spec, ring_mask, fov,
                                       t.edge_bins, t.edge_half_fov_deg,
                                       blocked_cells=g.dilate(res.above_mask, 1))
        clk['edge'] = time.perf_counter() - t4

        # --- 物体 (§8) ---------------------------------------------------
        t5 = time.perf_counter()
        found, obj_mask, _ = cl.extract(self.spec, h, fwd, left, h_r,
                                        ring_mask, t)
        best, _ = cl.select(found, self.p.match, t)
        res.clusters = found
        res.obj_mask = obj_mask
        res.selected = best
        clk['cluster'] = time.perf_counter() - t5

        res.spec = self.spec
        return self._finish(res, dt, clk, best)

    # ------------------------------------------------------------ 追尾と状態
    def _finish(self, res, dt, clk, best=None):
        t6 = time.perf_counter()
        meas = None if best is None else (best.fwd, best.left)
        pos = self.tracker.update(meas, dt)
        if best is not None:
            self._last_shape = (best.top_height, best.width,
                                0.5 * (best.top_height + best.height_min))
        if pos is not None:
            res.position = pos
            res.velocity = self.tracker.vel
            res.extrapolated = self.tracker.extrapolated
            res.top_height, res.width, res.height = self._last_shape
        clk['track'] = time.perf_counter() - t6

        # 状態は「壊れている方」を優先する。行動層は NO_OPPONENT では通常の
        # 探索に入り、ATTITUDE_STALE / RING_LOST では旋回を落とすなど別の扱いをする
        if res.ring_height is None or res.ring_mask is None or not res.ring_mask.any():
            res.status = RING_LOST
        elif self.attitude.stale:
            res.status = ATTITUDE_STALE
        elif res.position is None:
            res.status = NO_OPPONENT
        else:
            res.status = OK

        clk['total'] = sum(v for k, v in clk.items() if k != 'total')
        res.timings = clk
        return res
