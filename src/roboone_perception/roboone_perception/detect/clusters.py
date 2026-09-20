# -*- coding: utf-8 -*-
"""リング面より上の物体を取り出して、相手を 1 つ選ぶ。

docs/opponent_detection.pdf §8。

**リングの内側でだけ探す。** 制限を外すと、桜木町の bag で拾ったクラスタの 49% は
重心がリング外にあり (場外側は最大幅 p90 で 3.34 m、机や椅子の列)、厚木の bag では
100% がリング外だった。しかも「場外に物が多い」だけではなく、制限を外すと**リング上の
相手が背後の什器と 1 つの連結成分に融合して重心が場外へ引きずられる**。高さと
大きさのフィルタだけではこの融合は止められないので、「リングの内側に閉じてから
連結成分を取る」という順序そのものが効いている。

**外から差し込む塊は相手にしない** (docs/相手機の認識.md §6 の門 3。2026-09-20)。
レフリーの腕や、縁にしゃがんだ人の頭は、上端が低く幅も小さいので高さと大きさの
フィルタを通る。違いは「どこから生えているか」で、リングの上に立つ機体は足元が
リングの中にあるのに対し、人の腕はリングの外の胴体までつながっている。リングの縁の
外側の帯にある「面より上」のセルと 4 近傍でつながる塊は、外の物の一部として捨てる。
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
    #: この塊と 4 近傍でつながっている、リングの外の「面より上」のセル数
    outside_cells: int = 0
    #: 外から差し込んでいると判定したか (outside_cells >= tune.intrude_min_cells)
    intruding: bool = False

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


def extract(spec, h, fwd, left, h_r, ring_mask, tune, interior=None,
            beyond=None, outside_out=None):
    """リングの内側にある、面より上の塊を取り出す。

    interior は「リングの内側」のマスク (pipeline が凸包で作る)。None なら従来どおり
    見えたリング面 ring_mask を使う。どちらも ring_dilate_cells だけ膨張させた領域が
    候補の範囲になる。

    beyond は「確かにリングの外」にある面より上のセル (polar.beyond_floor_end)。
    None なら、縁の外側の帯にある面より上のセルをすべて外の物として数える。

    返り値は (clusters, obj_mask, labels)。obj_mask と labels はデバッグ表示用。
    outside_out に list を渡すと、外の物のマスクを 1 つ入れて返す (同じくデバッグ用)。
    """
    above = h - h_r
    sel = (above > tune.obj_h_lo) & (above < tune.obj_h_hi)
    if not np.any(sel):
        return [], np.zeros(spec.shape, dtype=bool), np.zeros(spec.shape, np.int32)

    hh, uu, vv = above[sel], fwd[sel], left[sel]
    iu, iv, inside = spec.index(uu, vv)
    # リングの内側を 1 セル膨張させた領域だけを候補にする (§8.1)
    base = ring_mask if interior is None else interior
    allowed = g.dilate(base, tune.ring_dilate_cells)
    # 縁の外側の帯にあって、面より上の点が十分あるセル。塊との接続をたどるのに使う。
    # そのうち「確かにリングの外」のものだけを外の物として数える
    link = outside = None
    if tune.intrude_min_cells > 0 and tune.intrude_band_cells > 0:
        cnt = np.bincount(iu[inside] * spec.nv + iv[inside],
                          minlength=spec.nu * spec.nv)
        band = g.dilate(allowed, tune.intrude_band_cells) & ~allowed
        link = (cnt.reshape(spec.shape) >= tune.intrude_cell_points) & band
        outside = link if beyond is None else (link & beyond)
        if outside_out is not None:
            outside_out.append(outside)
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
    # ラベルごとの最大・最小。ufunc.at は点数に比例して遅い (Pi 5 で 1 回 1 ms 級) ので、
    # ラベルで並べ替えて reduceat でまとめて取る
    order = np.argsort(lab, kind='stable')
    lab_s = lab[order]
    starts = np.flatnonzero(np.r_[True, lab_s[1:] != lab_s[:-1]])
    ids = lab_s[starts]
    hh_s, uu_s, vv_s = hh[order], uu[order], vv[order]
    top[ids] = np.maximum.reduceat(hh_s, starts)
    bot[ids] = np.minimum.reduceat(hh_s, starts)
    u_hi[ids] = np.maximum.reduceat(uu_s, starts)
    u_lo[ids] = np.minimum.reduceat(uu_s, starts)
    v_hi[ids] = np.maximum.reduceat(vv_s, starts)
    v_lo[ids] = np.minimum.reduceat(vv_s, starts)

    # 外の物との接続。内側の塊と外の物を 1 枚にしてラベルを振り直し、塊ごとに
    # 「同じ成分に入った外のセル数」を数える。重心などは内側の点だけで出したままなので、
    # つながっていても相手の位置が場外へ引かれることはない (§8.1 の順序は変えていない)
    out_cells = np.zeros(nlab, dtype=np.int64)
    if outside is not None and outside.any():
        ulabels, un = g.label_components(obj_mask | link)
        per_union = np.bincount(ulabels[outside], minlength=un + 1)
        ul_of_cell = ulabels[obj_mask]
        for k in range(1, nlab):
            mine = np.unique(ul_of_cell[cell_lab == k])
            out_cells[k] = int(per_union[mine].sum())

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
            outside_cells=int(out_cells[k]),
            intruding=bool(tune.intrude_min_cells > 0
                           and out_cells[k] >= tune.intrude_min_cells),
        ))
    return out, obj_mask, labels


def reject_reason(c, match, tune):
    """相手にしない理由を短い文字列で返す。通るなら None。

    passes() と同じ判定を、どの門で落ちたか分かる形にしたもの。テストランの
    ビューアが塊ごとに表示する (docs/相手機の認識.md §9)。
    """
    r = c.radius
    if c.n_cells < tune.min_cells:
        return 'セル数 %d < %d' % (c.n_cells, tune.min_cells)
    need = min_points_at(r, tune.min_points_n0, tune.min_points_r0,
                         tune.min_points_floor)
    if c.n_points < need:
        return '点数 %d < %.0f' % (c.n_points, need)
    if c.top_height < match.obj_top_min:
        return '低すぎる %.2f m' % c.top_height
    if c.top_height > match.obj_top_max:
        return '高すぎる %.2f m (人・什器)' % c.top_height
    if c.width < match.obj_width_min:
        return '細すぎる %.2f m' % c.width
    if c.width > match.obj_width_max:
        return '広すぎる %.2f m (什器)' % c.width
    if r > match.range_max:
        return '遠すぎる %.2f m' % r
    if c.intruding:
        return '外から差し込んでいる (外 %d セル)' % c.outside_cells
    return None


def passes(c, match, tune):
    """幾何フィルタ 式 (11)(12)(13) と、外から差し込む塊の棄却。"""
    return reject_reason(c, match, tune) is None


def select(clusters, match, tune):
    """条件を通ったもののうち最も近いものを相手とする。

    最大クラスタではなく最近接を採るのは、格闘競技では間合いの管理が先で、
    遠くの大きい塊よりも近くの小さい塊のほうが行動を決めるからである (§8.2)。
    """
    ok = [c for c in clusters if passes(c, match, tune)]
    if not ok:
        return None, ok
    return min(ok, key=lambda c: c.radius), ok
