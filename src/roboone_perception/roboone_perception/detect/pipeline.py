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
(roboone_walk_ref の walk_core と同じ流儀)。
"""

from dataclasses import dataclass, field
import math
import time

import numpy as np

from . import clusters as cl
from . import edge as ed
from . import grid as g
from . import ring as rg
from .attitude import AttitudeEstimator
from .geometry import (Deprojector, floor_visible_from, forward_ref,
                       ring_basis, to_plane)
from .params import DetectorParams
from .polar import PolarIndex
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
    plane_angle_deg: float = float('nan')   # 推定中の鉛直と面法線の食い違い (門は 12 度)
    stale_frames: int = 0              # 引き戻しが入らないまま進んだフレーム数
    n_points: int = 0

    # 相手 (追尾後)。status が OK でなくても、取れているときは埋める
    position: tuple = None             # (前方 u, 左 v) [m] リング平面座標
    velocity: tuple = (0.0, 0.0)       # [m/s]
    height: float = float('nan')       # [m] 重心のリング面からの高さ
    top_height: float = float('nan')   # [m] 上端 z_top
    width: float = float('nan')        # [m] max(w, d)
    #: z_top の絶対値から決めた種別 (clusters.ROBOT_STANDING / ROBOT_FALLEN)。
    #: 追尾が外挿している間は最後に観測した種別を保つ
    kind: str = None
    extrapolated: bool = False

    cliff: np.ndarray = None           # d_cliff(θ)。見えていない方位は NaN

    # --- 以下はデバッグ・テスト用 ----------------------------------------
    clusters: list = field(default_factory=list)
    selected: object = None
    ring_mask: np.ndarray = None
    interior_mask: np.ndarray = None   # リングの内側 (見えた面と自機の凸包)
    outside_mask: np.ndarray = None    # 縁の外の帯にある「外の物」
    seed_window: tuple = None          # (near, far) [m] 実際に使った種の窓
    obj_mask: np.ndarray = None
    obj_labels: np.ndarray = None      # セルごとの、一番上のボクセルが属する塊の番号
    above_mask: np.ndarray = None      # 面より上の全セル (リング外も含む)
    fov_cells: np.ndarray = None
    #: want_debug のときだけ。逆投影した点 [N,3] (カメラ座標) と、点ごとの塊の番号
    #: (Cluster.label、塊に入らない点は 0)。点の並びは Deprojector の画素の並び
    points: np.ndarray = None
    point_labels: np.ndarray = None
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
        self.polar = PolarIndex(self.spec)
        self._deproj = None
        self._seed = None
        self._seed_key = None
        self.seed_window = None
        #: 方位の基準にする機体前方 (カメラ座標)。光軸ではない (geometry の注記)
        self._fwd_ref = forward_ref(self.p.body.cam_pitch_deg)
        #: 追尾が外挿している間は、最後に観測した寸法と種別を保つ
        self._last_shape = (float('nan'), float('nan'), float('nan'))
        self._last_kind = None
        self.frames = 0

    def reset_reference(self, accel=None):
        """基準姿勢を取り直す (AttitudeEstimator.reset の注記)。追尾も捨てる。

        accel は静止中の加速度の平均。None なら取り付けから決まる鉛直へ戻す
        (水平付けでホーム姿勢に立っていれば、それが正しい値に近い)。
        """
        ok = self.attitude.reset(up=self.p.body.up_from_mount, accel=accel)
        self.tracker.reset()
        self._last_shape = (float('nan'), float('nan'), float('nan'))
        return ok

    # ---------------------------------------------------------------- 種
    def _seed_mask(self, intr=None):
        """リング成分の種にするセル (params.TuneParams.seed_* の注記を参照)。

        seed_auto のときは、床が写り始める距離 (取り付けと画角で決まる) の先に窓を
        置く。水平付けのカメラでは固定の 0.15〜0.70 m に床が 1 画素も写らない。
        """
        t = self.p.tune
        key = None
        if t.seed_auto and intr is not None:
            key = (intr.height, round(intr.fy, 3), round(intr.cy, 3))
        if self._seed is None or key != self._seed_key:
            near, far = t.seed_near, t.seed_far
            if key is not None:
                start = floor_visible_from(intr, self.p.body.cam_height,
                                           self.p.body.cam_pitch_deg)
                if math.isfinite(start):
                    near = max(t.seed_near, start + t.cell)
                    far = near + (t.seed_far - t.seed_near)
            iu = np.arange(self.spec.nu)[:, None]
            iv = np.arange(self.spec.nv)[None, :]
            fwd, left = self.spec.centers(iu, iv)
            self._seed = ((fwd >= near) & (fwd <= far)
                          & (np.abs(left) <= t.seed_half_width))
            self._seed_key = key
            self.seed_window = (near, far)
        return self._seed

    def _deprojector(self, intr):
        t = self.p.tune
        if self._deproj is None or not self._deproj.matches(intr, t.stride,
                                                            t.border_px):
            self._deproj = Deprojector(intr, t.stride, t.border_px)
        return self._deproj

    def _bridge_shadows(self, ring_mask, labels, n, pick, occ, above_mask,
                        rel_h, fwd, left):
        """相手の影で分断された床を、影をまたいでリングへつなぎ直す。

        近い相手は見えている床を左右 2 つに分け、種の窓も影に入る。種の成分 (か最大の
        成分) だけをリングにすると片側しか拾えず、「リングの内側」の凸包が相手を
        覆わなくなる。そこで、影と物のセルを通れば種の成分へ届く床の成分を足す。

        足してよいのは**純粋な床**の成分だけ。リングの外に立つ人や什器の垂直な面は
        リング面の高さを横切るので床の帯に点を落とすが、同じセルに面より上か下の点も
        必ず持つ。それが半分を超える成分や、小さすぎる成分は床ではないので足さない
        (足すと凸包が場外の人まで伸びて、人が相手の候補に入る)。
        """
        t = self.p.tune
        flat = labels.ravel()
        size = np.bincount(flat, minlength=n + 1)
        others = size.copy()
        others[[0, pick]] = 0
        if others.max() < t.bridge_min_cells:
            return ring_mask            # 足す候補が無い。影のラベリング (数 ms) を省く
        low = rel_h < -t.below_band
        below = g.count_cells(self.spec, fwd[low], left[low]) >= 3
        bridge = (self.polar.shadow(above_mask, t.shadow_bridge)
                  | above_mask) & ~below
        joined, _ = g.label_components(occ | bridge)
        root = joined[ring_mask]
        if root.size == 0:
            return ring_mask
        root = int(np.bincount(root).argmax())
        impure = np.bincount(flat, weights=(above_mask | below).ravel(),
                             minlength=n + 1)
        uniq, first = np.unique(flat, return_index=True)
        # **成分ごとに labels == k を回さない** (2026-09-24)。1 回の比較が格子ぜんぶを
        # 舐めるので、成分の数に比例して重くなる (90x120 の格子で 5 成分 0.05 ms、
        # 200 成分 1.2 ms、2000 成分 11 ms。格子を細かくすればその 4 倍)。
        # 足す成分を先に選んでから np.isin で 1 回にまとめる (2000 成分で 0.5 ms)。
        # ※ 実機で成分が数千個になる場面は**まだ確認していない**。合成シーンでは
        #   姿勢を 30 度外しても成分は 1 個のままで、ここは重くならなかった
        take = []
        for k, i0 in zip(uniq.tolist(), first.tolist()):
            if k in (0, pick) or size[k] < t.bridge_min_cells:
                continue
            if joined.ravel()[i0] != root or impure[k] > 0.5 * size[k]:
                continue
            take.append(k)
        out = ring_mask.copy()
        if take:
            out |= np.isin(labels, take)
        return out

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
            want_debug   表示のための中間結果も残すか。true なら点と点ごとの塊の番号
                         (res.points / res.point_labels) も持たせる
        """
        t = self.p.tune
        res = DetectionResult()
        clk = {}
        t0 = time.perf_counter()

        # --- 逆投影 (§4) -----------------------------------------------
        pts, border = self._deprojector(intr)(depth, depth_scale,
                                              t.depth_min, t.depth_max)
        res.n_points = int(pts.shape[0])
        if want_debug:
            res.points = pts
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
        occ = g.close(occ, t.morph_cells)
        # 面より上のセル。ここでは影を作るものとして、あとで縁と物体でも使う
        above = (h - h_r > t.obj_h_lo) & (h - h_r < t.obj_h_hi)
        above_mask = g.count_cells(self.spec, fwd[above], left[above]) > 0
        res.above_mask = above_mask
        labels, n = g.label_components(occ)
        pick = g.component_of_seed(labels, n, self._seed_mask(intr),
                                   t.seed_fallback_to_largest)
        res.seed_window = self.seed_window
        ring_mask = (labels == pick) if pick else np.zeros(self.spec.shape, bool)
        if pick and n > 1 and t.shadow_bridge > 0 and above_mask.any():
            ring_mask = self._bridge_shadows(ring_mask, labels, n, pick, occ,
                                             above_mask, h - h_r, fwd, left)
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
        res.cliff = ed.cliff_distances(self.spec, ring_mask, fov,
                                       t.edge_bins, t.edge_half_fov_deg,
                                       blocked_cells=g.dilate(res.above_mask,
                                                              t.morph_cells))
        clk['edge'] = time.perf_counter() - t4

        # --- 物体 (§8) ---------------------------------------------------
        t5 = time.perf_counter()
        # リングの内側 = 見えたリング面と自機の位置の凸包 (params.ring_hull の注記)
        interior = None
        if t.ring_hull:
            iu0, iv0, ok0 = self.spec.index(np.array([0.0]), np.array([0.0]))
            origin = [(int(iu0[0]), int(iv0[0]))] if ok0[0] else []
            interior = g.convex_hull_mask(ring_mask, origin)
        res.interior_mask = interior
        outside = []
        plabels = [] if want_debug else None
        beyond = self.polar.beyond_floor_end(res.above_mask, ring_mask)
        found, obj_mask, obj_labels = cl.extract(
            self.spec, h, fwd, left, h_r, ring_mask, t, interior=interior,
            beyond=beyond, outside_out=outside, point_labels_out=plabels)
        res.outside_mask = outside[0] if outside else None
        res.point_labels = plabels[0] if plabels else None
        best, _ = cl.select(found, self.p.match, t)
        res.clusters = found
        res.obj_mask = obj_mask
        res.obj_labels = obj_labels
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
            self._last_kind = best.kind
        if pos is not None:
            res.position = pos
            res.velocity = self.tracker.vel
            res.extrapolated = self.tracker.extrapolated
            res.top_height, res.width, res.height = self._last_shape
            res.kind = self._last_kind
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

        res.plane_angle_deg = math.degrees(self.attitude.last_angle)
        res.stale_frames = int(self.attitude.since_correction)
        clk['total'] = sum(v for k, v in clk.items() if k != 'total')
        res.timings = clk
        return res
