# -*- coding: utf-8 -*-
"""リング境界から d_cliff(θ) を作る。

docs/opponent_detection.pdf §7。物体クラスタリングと同じ占有グリッドの副産物として
出すのが方針で、エッジ検出のために別のアルゴリズムを足さない。

**視野の縁とリングの端を区別する。** 第 44 回競技規則のリングは 360 cm 角の四隅を
90 cm 落とした八角形で、俯角 30°・高さ 0.35 m のカメラから見える横幅
(1 m 先で 1.72 m、2 m 先で 3.15 m) よりつねに広い。中央に立てば左右の端は視野の外に
あるので、左右方向で見つかる「境界」は多くの場合リングの端ではなく見えている範囲の
端である。そのままの値を載せると、リングの中央にいるのに縁が近いと誤って伝わる。
床が切れたのを見たときだけ有限値を出し、見えていないことは NaN で伝える。

(この判定は元文書では未実装として残されていた。ここでは逆投影の段で
「元画素が視野の縁にあるか」を点ごとに持ち回り、セルへ伝播させて実装している。)


文書からもう 1 つ変えたところ: 方位ビンの代表を「最も近い境界セル」ではなく
「最も遠いリングセル」にしてある。俯角 30 度・高さ 0.35 m では水平 0.188 m より
近い面が写らないので、リング成分の内側の縁（＝足元の死角との境目）は常に境界セルに
なる。それを代表に採ると、どの方位でも先に死角の縁が当たってしまい、その先にある
本物の切れ目が出てこない（実際、リングの前端 0.5 m を見ている合成シーンで、
前方のビンがほぼ全部 NaN になった）。

外向きに最も遠いセルを採れば、死角の縁は自然に無視され、相手に隠れて途中が
欠けても（隣の方位経由で連結が保たれるので）その先の縁を拾える。

残るのが**遮蔽の影**である。相手が縁まで影を落としている方位では、床は確かに
そこで切れて見えるが、それは崖ではない。合成シーンでは 1.0 m の前端が 0.55 m の
「崖」として出た。行動層はこれを縁として避けるので、相手に近づけなくなる。
影と崖は区別できる — 影の手前には必ず面より上の塊がある。よって、代表セルが
面より上のセルに radially 隣接していたら NaN にする。「見えていないことは
見えていないと伝える」という §7 の方針そのままの扱いである。
"""

import math

import numpy as np


def cliff_distances(spec, ring_mask, fov_cells, bins=64, half_fov_deg=45.0,
                    blocked_cells=None):
    """方位ビンごとの床の切れ目までの距離 d_cliff(θ) を返す。

    引数:
        spec       グリッドの定義
        ring_mask  リング成分 (連結成分で選んだ後のもの)
        fov_cells  視野の縁に接するセル。ここが代表になった方位は NaN
        blocked_cells 面より上の塊に隣接するセル。遮蔽の影を崖と読まないための除外
        bins       方位の刻み数
        half_fov_deg 覆う方位の半幅。ビン k の中心は
                   θ_k = -half + (k+0.5)·2·half/bins

    返り値は長さ bins の float32 配列。見えていない方位は NaN。
    """
    from . import grid as g
    out = np.full(bins, np.nan, dtype=np.float32)
    # 外向きに最も遠いセルは必ず境界セルなので、境界だけ見れば足りる。
    # リング全面 (千数百セル) ではなく外周 (百数十セル) で済む
    iu, iv = np.nonzero(g.boundary(ring_mask))
    if iu.size == 0:
        return out

    fwd, left = spec.centers(iu.astype(np.float64), iv.astype(np.float64))
    r = np.hypot(fwd, left)
    beta = np.degrees(np.arctan2(left, fwd))
    on_fov = fov_cells[iu, iv]
    if blocked_cells is not None:
        on_fov = on_fov | blocked_cells[iu, iv]

    # 1 つのセルが張る方位の幅。近いセルほど広い方位を覆う。ビン幅 (64 分割で
    # 1.4 度) は 0.5 m 先のセルが張る 4 度よりずっと細いので、セル中心の方位だけで
    # ビンに入れると、間のビンが空いたまま残って遠近が混ざる
    half_diag = 0.7072 * spec.cell
    spread = np.degrees(np.arctan2(half_diag, np.maximum(r, 1e-3)))

    half = float(half_fov_deg)
    step = 2.0 * half / bins
    k_lo = np.floor((beta - spread + half) / step).astype(np.int64)
    k_hi = np.floor((beta + spread + half) / step).astype(np.int64)
    np.clip(k_lo, 0, bins - 1, out=k_lo)
    np.clip(k_hi, 0, bins - 1, out=k_hi)
    visible = (beta + spread >= -half) & (beta - spread <= half)

    # 遠い順に、まだ埋まっていないビンを埋める
    seen = np.zeros(bins, dtype=bool)
    half_cell = 0.5 * spec.cell
    for i in np.argsort(-r):
        if not visible[i]:
            continue
        sl = slice(int(k_lo[i]), int(k_hi[i]) + 1)
        m = ~seen[sl]
        if not m.any():
            continue
        seen[sl] = True
        if not on_fov[i]:
            # セル中心ではなく、そのセルの外側の端までを切れ目とする
            out[sl][m] = r[i] + half_cell
    return out


def bin_bearings(bins=64, half_fov_deg=45.0):
    """各ビンの中心方位 [rad]。受け側 (behavior) と約束を共有するための関数。"""
    half = math.radians(half_fov_deg)
    step = 2.0 * half / bins
    return np.array([-half + (k + 0.5) * step for k in range(bins)],
                    dtype=np.float32)
