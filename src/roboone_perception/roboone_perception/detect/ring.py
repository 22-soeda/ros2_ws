# -*- coding: utf-8 -*-
"""リング面の高さ h_r と、その法線。

docs/opponent_detection.pdf §6。ここで最大平面 (素の RANSAC) を選んではいけない。

桜木町の bag では、リング面 (カメラ下 0.338 m) と場外の床 (0.662 m) がどちらも
「点群の中の大きな平面」で、視野の取り合いでどちらが最大にもなる。99 フレームの
うち 14 フレームで場外の床に貼り付き、1 フレームでの飛びは最大 333 mm になった。
貼り替えは単発のノイズではなく数秒続く状態遷移なので、平滑化しても意味がない
(むしろ 2 つの峰の間の、存在しない高さに面を置く)。

代わりに、カメラは機体に固定されていてリング面までの高さが設計値 h_cam として
既知であることを使い、高さヒストグラムのうち -h_cam の近くにある層を採る。
同じフレーム列で標準偏差 9 mm、50 mm を超える飛びは 0 回になり、計算も
ヒストグラム 1 回で済む。
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class PlaneFit:
    """面あてはめの結果。ok=False のときは normal / resid を見ない。"""

    ok: bool
    normal: np.ndarray = None
    resid: float = float('inf')
    n_points: int = 0


def ring_height(h, cam_height, window=0.25, bin_w=0.010, refine=0.020,
                min_points=300):
    """高さヒストグラムから h_r を出す。式 (9)。

    -h_cam の周り ±window に限った中の最頻ビンを採り、その ±refine にある点の
    平均で精密化する。窓を ±0.25 m に取るのは機体が沈む・傾く分の余裕で、
    リングの段差 34 cm に対しては場外の床が窓に入らない幅でもある。

    返り値は (h_r, 窓の中の点数)。点数が min_points に満たなければ (None, 数)。
    """
    lo, hi = -cam_height - window, -cam_height + window
    sel = h[(h > lo) & (h < hi)]
    if sel.size < min_points:
        return None, int(sel.size)
    nbins = max(1, int(round((hi - lo) / bin_w)))
    counts, edges = np.histogram(sel, bins=nbins, range=(lo, hi))
    c = 0.5 * (edges[np.argmax(counts)] + edges[np.argmax(counts) + 1])
    near = sel[np.abs(sel - c) < refine]
    return (float(near.mean()) if near.size else float(c)), int(sel.size)


def fit_plane(points, h, fwd, left, h_r, band=0.05, radius=1.5, min_points=200,
              max_points=4000):
    """リング面に平面をあてはめて、法線と残差を返す。

    §5.4 で踏んだ 2 つの罠をそのまま避ける形にしてある。

    * **締め直しを点群全体に開かない。** 残差の小さい点を選び直す反復を全体に
      開くと、背景の床や什器を斜めに貫く平面の方がインライアを多く稼ぐので、
      数回でリング面から離れる (厚木の bag では残差 1.6 mm のまま 22.8° ずれた)。
      選び直しは最初の高さ帯の中に閉じる。
    * **帯を遠方まで伸ばさない。** 厚さ 5 cm の帯を 4 m 先まで含めると、0.7°
      傾けただけで反対側の縁に届いてしまい、帯を斜めに貫く平面が同じ残差で通る。
      水平距離 1.5 m 以内に限る。1.5 m の広がりに対する 2 mm の残差なら、
      傾きは 0.1° 級で決まる。

    法線は主成分分析の最小固有ベクトル (全最小二乗) で取る。高さを (u,v) の
    関数として最小二乗で解く形より、面が傾いたときの偏りが小さい。

    点数が max_points を超える分は等間隔に間引く。乱数ではなく等間隔なのは、
    同じ入力から同じ出力が出る性質を保つため。
    """
    r2 = fwd * fwd + left * left
    sel = (np.abs(h - h_r) < band) & (r2 < radius * radius)
    n = int(np.count_nonzero(sel))
    if n < min_points:
        return PlaneFit(False, n_points=n)
    idx = np.flatnonzero(sel)
    if idx.size > max_points:
        idx = idx[::int(np.ceil(idx.size / max_points))]
    p = points[idx].astype(np.float64)
    c = p.mean(axis=0)
    q = p - c
    # 3x3 の共分散なので eigh で足りる (SVD より速い)
    w, v = np.linalg.eigh(q.T @ q / q.shape[0])
    normal = v[:, 0]
    resid = float(np.sqrt(max(w[0], 0.0)))
    # 締め直しは 1 回だけ、しかも最初に選んだ帯の中に閉じる
    d = q @ normal
    keep = np.abs(d) < max(3.0 * resid, 0.003)
    if int(np.count_nonzero(keep)) >= min_points:
        p2 = p[keep]
        c2 = p2.mean(axis=0)
        q2 = p2 - c2
        w, v = np.linalg.eigh(q2.T @ q2 / q2.shape[0])
        normal = v[:, 0]
        resid = float(np.sqrt(max(w[0], 0.0)))
        n = int(p2.shape[0])
    return PlaneFit(True, normal=normal, resid=resid, n_points=n)
