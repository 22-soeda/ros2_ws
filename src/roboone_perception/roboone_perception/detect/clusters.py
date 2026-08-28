# -*- coding: utf-8 -*-
"""リング面より上の物体を取り出して、相手を 1 つ選ぶ。

docs/opponent_detection.pdf §8。

**リングの内側でだけ探す。** 制限を外すと、桜木町の bag で拾ったクラスタの 49% は
重心がリング外にあり (場外側は最大幅 p90 で 3.34 m、机や椅子の列)、厚木の bag では
100% がリング外だった。しかも「場外に物が多い」だけではなく、制限を外すと**リング上の
相手が背後の什器と 1 つの連結成分に融合して重心が場外へ引きずられる**。高さと
大きさのフィルタだけではこの融合は止められないので、「リングの内側に閉じてから
連結成分を取る」という順序そのものが効いている。
"""

from dataclasses import dataclass
import math

import numpy as np

from . import grid as g


@dataclass
class Cluster:
    """リング上の 1 つの塊。距離・方位はリング平面座標 (カメラ原点) のもの。"""

    label: int
    n_points: int
    n_cells: int
    fwd: float          # [m] 重心 u
    left: float         # [m] 重心 v
    top_height: float   # [m] 上端のリング面からの高さ z_top
    width: float        # [m] 水平方向の広がり max(w, d)
    height_min: float   # [m] 下端のリング面からの高さ (デバッグ用)

    @property
    def radius(self):
        """[m] 水平距離 r = sqrt(u² + v²)。名前を range にすると組み込みを隠す。"""
        return math.hypot(self.fwd, self.left)

    @property
    def bearing(self):
        return math.atan2(self.left, self.fwd)


def min_points_at(r, n0, r0, floor):
    """距離 r で要求する最小点数 N_min(r) = N0 (r0/r)^2。式 (14) から。

    高さ H・幅 W の物体が距離 r にあるとき、間引き幅 d の画像に写る画素数は
    N ≃ fx fy H W / (d² r²) で距離の 2 乗に反比例する。固定の下限を置くと
    遠方で先に落ちるので、しきい値も同じ形にする。
    """
    if r <= 1e-3:
        return floor
    return max(floor, n0 * (r0 / r) ** 2)


def extract(spec, h, fwd, left, h_r, ring_mask, tune):
    """リング成分の内側にある、面より上の塊を取り出す。

    返り値は (clusters, obj_mask, labels)。obj_mask と labels はデバッグ表示用。
    """
    above = h - h_r
    sel = (above > tune.obj_h_lo) & (above < tune.obj_h_hi)
    if not np.any(sel):
        return [], np.zeros(spec.shape, dtype=bool), np.zeros(spec.shape, np.int32)

    hh, uu, vv = above[sel], fwd[sel], left[sel]
    iu, iv, inside = spec.index(uu, vv)
    # リング成分を 1 セル膨張させた領域の内側だけを候補にする (§8.1)
    allowed = g.dilate(ring_mask, tune.ring_dilate_cells)
    keep = inside.copy()
    keep[inside] &= allowed[iu[inside], iv[inside]]
    if not np.any(keep):
        return [], np.zeros(spec.shape, dtype=bool), np.zeros(spec.shape, np.int32)

    hh, uu, vv = hh[keep], uu[keep], vv[keep]
    iu, iv = iu[keep], iv[keep]
    obj_mask = np.zeros(spec.shape, dtype=bool)
    obj_mask[iu, iv] = True
    labels, n = g.label_components(obj_mask)
    if n == 0:
        return [], obj_mask, labels

    lab = labels[iu, iv]
    nlab = n + 1
    n_points = np.bincount(lab, minlength=nlab)
    cell_lab = labels[obj_mask]
    n_cells = np.bincount(cell_lab, minlength=nlab)
    sum_u = np.bincount(lab, weights=uu, minlength=nlab)
    sum_v = np.bincount(lab, weights=vv, minlength=nlab)
    top = np.full(nlab, -np.inf)
    bot = np.full(nlab, np.inf)
    u_hi = np.full(nlab, -np.inf)
    u_lo = np.full(nlab, np.inf)
    v_hi = np.full(nlab, -np.inf)
    v_lo = np.full(nlab, np.inf)
    np.maximum.at(top, lab, hh)
    np.minimum.at(bot, lab, hh)
    np.maximum.at(u_hi, lab, uu)
    np.minimum.at(u_lo, lab, uu)
    np.maximum.at(v_hi, lab, vv)
    np.minimum.at(v_lo, lab, vv)

    out = []
    for k in range(1, nlab):
        if n_points[k] == 0:
            continue
        out.append(Cluster(
            label=k,
            n_points=int(n_points[k]),
            n_cells=int(n_cells[k]),
            fwd=float(sum_u[k] / n_points[k]),
            left=float(sum_v[k] / n_points[k]),
            top_height=float(top[k]),
            height_min=float(bot[k]),
            # 広がりは「点の広がり」ではなく「セルの広がり」に合わせて 1 セル足す。
            # 2 点しかない塊の幅が 0 になって wmin で落ちるのを防ぐため
            width=float(max(u_hi[k] - u_lo[k], v_hi[k] - v_lo[k]) + spec.cell),
        ))
    return out, obj_mask, labels


def passes(c, match, tune):
    """幾何フィルタ 式 (11)(12)(13)。"""
    r = c.radius
    return (c.n_cells >= tune.min_cells
            and c.n_points >= min_points_at(r, tune.min_points_n0,
                                            tune.min_points_r0,
                                            tune.min_points_floor)
            and match.obj_top_min <= c.top_height <= match.obj_top_max
            and match.obj_width_min <= c.width <= match.obj_width_max
            and r <= match.range_max)


def select(clusters, match, tune):
    """条件を通ったもののうち最も近いものを相手とする。

    最大クラスタではなく最近接を採るのは、格闘競技では間合いの管理が先で、
    遠くの大きい塊よりも近くの小さい塊のほうが行動を決めるからである (§8.2)。
    """
    ok = [c for c in clusters if passes(c, match, tune)]
    if not ok:
        return None, ok
    return min(ok, key=lambda c: c.radius), ok
