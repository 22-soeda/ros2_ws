# -*- coding: utf-8 -*-
"""リング平面上の占有グリッドと、その上の連結成分。

docs/opponent_detection.pdf §7。エッジ抽出と物体クラスタリングが同じグリッドを
共有するのがこの段の要点で、エッジのために別のアルゴリズムを足さない。

numpy で書いてある。**連結成分だけは cv2 があればそちらを使う** (2026-09-23。
§10 の 3 番目の手をここで使った)。セルを 2.5 cm にするとグリッドは 180x240 =
43200 になり、1 フレームに 2 回まわる 4 近傍のラベリングが段の中で一番重くなる
(numpy 版で 2.4 ms + 5.3 ms)。cv2 は C の実装で、同じ大きさの最悪ケースでも
0.46 ms。**cv2 が無ければ numpy 版に落ちるので、依存としては任意のまま。**
それ以外 (モルフォロジ・凸包・ボクセルの 26 近傍) は numpy のままで、
scipy.ndimage は使わない。
"""

from dataclasses import dataclass

import numpy as np

try:                                    # 連結成分だけ C の実装に逃がす (下の注記)
    import cv2 as _cv2
except ImportError:                     # pragma: no cover
    _cv2 = None


@dataclass(frozen=True)
class GridSpec:
    """(u, v) 平面を覆う格子。u が前方、v が左。"""

    cell: float
    u_min: float
    u_max: float
    v_min: float
    v_max: float

    @property
    def nu(self):
        return max(1, int(round((self.u_max - self.u_min) / self.cell)))

    @property
    def nv(self):
        return max(1, int(round((self.v_max - self.v_min) / self.cell)))

    @property
    def shape(self):
        return (self.nu, self.nv)

    def index(self, fwd, left):
        """(u, v) → (行, 列, 格子内か)。行が前方、列が左。"""
        iu = np.floor((fwd - self.u_min) / self.cell).astype(np.int32)
        iv = np.floor((left - self.v_min) / self.cell).astype(np.int32)
        inside = (iu >= 0) & (iu < self.nu) & (iv >= 0) & (iv < self.nv)
        return iu, iv, inside

    def centers(self, iu, iv):
        """セル添字 → セル中心の (u, v)。"""
        return (self.u_min + (iu + 0.5) * self.cell,
                self.v_min + (iv + 0.5) * self.cell)


def count_cells(spec, fwd, left, weights=None):
    """点をセルに落として、セルごとの点数を返す。"""
    iu, iv, inside = spec.index(fwd, left)
    flat = iu[inside] * spec.nv + iv[inside]
    w = None if weights is None else np.asarray(weights)[inside]
    counts = np.bincount(flat, weights=w, minlength=spec.nu * spec.nv)
    return counts.reshape(spec.shape)


def any_cells(spec, fwd, left, flag):
    """視野の縁の伝播用。flag が立った点を 1 つでも含むセルに True を立てる。"""
    return count_cells(spec, fwd[flag], left[flag]) > 0


def dilate(mask, iters=1):
    """4 近傍の膨張。"""
    out = mask
    for _ in range(iters):
        m = out
        d = m.copy()
        d[1:, :] |= m[:-1, :]
        d[:-1, :] |= m[1:, :]
        d[:, 1:] |= m[:, :-1]
        d[:, :-1] |= m[:, 1:]
        out = d
    return out


def erode(mask, iters=1):
    """4 近傍の収縮。格子の外は「空」とみなすので、外周のセルは必ず削れる。"""
    out = mask
    for _ in range(iters):
        m = out
        e = m.copy()
        e[1:, :] &= m[:-1, :]
        e[:-1, :] &= m[1:, :]
        e[:, 1:] &= m[:, :-1]
        e[:, :-1] &= m[:, 1:]
        e[0, :] = False
        e[-1, :] = False
        e[:, 0] = False
        e[:, -1] = False
        out = e
    return out


def close(mask, iters=1):
    """膨張してから収縮。depth の抜けで開いた 1 セルの穴を塞ぐ (§7)。

    穴を塞ぐのが目的なので、収縮の側で外周を削らないように 1 セルだけ余白を
    取ってから処理する。そうしないと閉じるたびにリングが 1 周ぶん痩せる。
    """
    pad = np.zeros((mask.shape[0] + 2, mask.shape[1] + 2), dtype=bool)
    pad[1:-1, 1:-1] = mask
    pad = erode(dilate(pad, iters), iters)
    return pad[1:-1, 1:-1]


def label_components(mask):
    """4 近傍の連結成分ラベリング。返り値は (labels, n)。背景は 0。

    **cv2 があればそれを使う** (2026-09-23)。ここは段の中で一番重く、セルを細かく
    すると占有セル数が 1/cell^2 で増えて頭打ちになる。cv2 は C で書かれていて、
    180x240 の最悪ケース (乱数) でも 0.46 ms で終わる。

    cv2 が無い環境では下の numpy 版に落ちる (依存は numpy だけ、という性質は
    そのまま)。**番号の振り方は違ってよい。** 呼ぶ側はどれも大きさや種で選んで
    いて、番号そのものには依らない。両者が同じ分け方をすることは単体テストで
    見ている (test_label_components_matches_a_reference)。
    """
    if _cv2 is not None:
        n, labels = _cv2.connectedComponents(
            np.ascontiguousarray(mask, dtype=np.uint8), connectivity=4)
        return labels.astype(np.int32), int(n) - 1
    return _label_components_numpy(mask)


def _label_components_numpy(mask):
    """cv2 が無いときの 4 近傍ラベリング。隣接ペアを numpy で作って union-find。"""
    labels = np.zeros(mask.shape, dtype=np.int32)
    idx = np.flatnonzero(mask.ravel())
    if idx.size == 0:
        return labels, 0
    # 占有セルへの通し番号
    order = np.full(mask.size, -1, dtype=np.int32)
    order[idx] = np.arange(idx.size, dtype=np.int32)

    _, nv = mask.shape
    pairs = []
    # 縦の隣接 (行方向)
    a = mask[:-1, :] & mask[1:, :]
    if a.any():
        r, c = np.nonzero(a)
        pairs.append(np.stack([order[r * nv + c], order[(r + 1) * nv + c]], axis=1))
    # 横の隣接 (列方向)
    b = mask[:, :-1] & mask[:, 1:]
    if b.any():
        r, c = np.nonzero(b)
        pairs.append(np.stack([order[r * nv + c], order[r * nv + c + 1]], axis=1))

    inv, n = _union_find(idx.size, pairs)
    labels.ravel()[idx] = inv + 1
    return labels, n


def _union_find(n_nodes, pairs):
    """隣接ペアから連結成分のラベルを振る。返り値は (0 始まりのラベル, 成分数)。

    **全部 numpy で回す** (2026-09-23)。以前は Python の union-find で、ペアを
    1 組ずつ触っていた。セルを細かくすると占有セル数が 1/cell^2 で増えて
    ここが頭打ちになる (0.05 で 1.8 ms が 0.025 で 7.7 ms)。

    やっているのは「隣へ最小ラベルを配る」と「ポインタを根まで跳ね上げる」の
    繰り返しで、どちらも配列 1 本の演算。ラベルは単調に減るだけなので必ず止まり、
    跳ね上げがあるので回数は成分の直径の対数で収まる。
    """
    if n_nodes == 0:
        return np.zeros(0, dtype=np.int32), 0
    if not pairs:
        return np.arange(n_nodes, dtype=np.int32), n_nodes

    e = np.concatenate(pairs, axis=0)
    # 両向きに並べて、送り先 a でまとめられるように一度だけ並べ替える
    a = np.concatenate([e[:, 0], e[:, 1]])
    b = np.concatenate([e[:, 1], e[:, 0]])
    order = np.argsort(a, kind='stable')
    a_s, b_s = a[order], b[order]
    starts = np.flatnonzero(np.r_[True, a_s[1:] != a_s[:-1]])
    heads = a_s[starts]

    lab = np.arange(n_nodes, dtype=np.int64)
    while True:
        # 隣が持っている最小のラベルを受け取る
        new = lab.copy()
        new[heads] = np.minimum(lab[heads], np.minimum.reduceat(lab[b_s], starts))
        # ポインタを根まで跳ね上げる (1 回だけだと反復が増えて、かえって遅い)
        while True:
            nxt = new[new]
            if np.array_equal(nxt, new):
                break
            new = nxt
        if np.array_equal(new, lab):
            break
        lab = new

    uniq, inv = np.unique(lab, return_inverse=True)
    return inv.astype(np.int32).reshape(-1), int(uniq.size)


#: 3 次元の 26 近傍のうち、重複しない 13 方向 (残りは符号を反転したもの)
_NB26 = tuple((du, dv, dw)
              for du in (-1, 0, 1) for dv in (-1, 0, 1) for dw in (-1, 0, 1)
              if (du, dv, dw) > (0, 0, 0))


def label_voxels(vox, shape):
    """占有ボクセルを 26 近傍でつないでラベルを振る (§8。2026-09-23)。

    引数:
        vox    (M, 3) の整数ボクセル添字 (行 = 前方 u, 列 = 左 v, 段 = 高さ z)。
               重複の無いものを渡すこと
        shape  (nu, nv, nw)

    返り値は (labels, n) で、labels は vox と同じ並びの 1 始まりのラベル。

    **高さを結合の条件に入れるためにこれを使う。** 真上から見た 2 次元の連結では、
    相手の真上に浮いたノイズの 1 点が同じ塊に入り、その高さが上端になってしまう
    (実機で「人・什器」に化ける)。ボクセルで切ると、離れて浮いたものは別の塊に
    なり、点数とセル数の門で落ちる。

    密な 3 次元配列は作らない。占有は物の表面ぶんしか無い (数百) ので、
    ボクセルの通し番号を searchsorted で引くほうが速くて場所も食わない。
    """
    m = len(vox)
    if m == 0:
        return np.zeros(0, dtype=np.int32), 0
    nu, nv, nw = shape
    vox = np.asarray(vox, dtype=np.int64)
    key = (vox[:, 0] * nv + vox[:, 1]) * nw + vox[:, 2]
    order = np.argsort(key, kind='stable')
    key_sorted = key[order]

    pairs = []
    src = np.arange(m, dtype=np.int32)
    for off in _NB26:
        nb = vox + off
        ok = np.ones(m, dtype=bool)
        for ax, hi in enumerate(shape):
            ok &= (nb[:, ax] >= 0) & (nb[:, ax] < hi)
        if not ok.any():
            continue
        nk = (nb[ok, 0] * nv + nb[ok, 1]) * nw + nb[ok, 2]
        pos = np.searchsorted(key_sorted, nk)
        hit = pos < m
        pos = np.minimum(pos, m - 1)
        hit &= key_sorted[pos] == nk
        if not hit.any():
            continue
        pairs.append(np.stack([src[ok][hit], order[pos[hit]].astype(np.int32)],
                              axis=1))

    inv, n = _union_find(m, pairs)
    return inv + 1, n


def component_of_seed(labels, n_labels, seed_mask, fallback_to_largest=True):
    """種の窓に最も多くのセルを持つ成分の番号を返す。無ければ 0 か最大成分。

    §7 の「最大の成分ではなく自分が乗っている成分を選ぶ」がここ。会場のもっと
    広い床が見えていてもリングを取り違えないための選び方で、リング面の高さが
    場外の床と紛らわしいときの最後の砦になる。
    """
    if n_labels == 0:
        return 0
    seeded = labels[seed_mask & (labels > 0)]
    if seeded.size:
        counts = np.bincount(seeded, minlength=n_labels + 1)
        return int(np.argmax(counts))
    if not fallback_to_largest:
        return 0
    counts = np.bincount(labels.ravel(), minlength=n_labels + 1)
    counts[0] = 0
    return int(np.argmax(counts))


def boundary(mask):
    r"""成分の境界セル E = R \ erode(R, 1)。式 (10)。"""
    return mask & ~erode(mask, 1)


def _hull(points):
    """2 次元の凸包 (Andrew の monotone chain)。points は (r, c) のタプルの列。"""
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def convex_hull_mask(mask, extra_cells=()):
    """凸包を塗ったマスクを返す。対象は mask の True セルと extra_cells [(iu, iv), ...]。

    「リングの内側」を作るのに使う (docs/相手機の認識.md §3)。見えたリング面は
    相手の影や足元の死角で欠けるが、リングは凸で自機はその上に立っているので、
    見えた面と自機の位置の凸包はリングの中に収まる。

    凸包の頂点になりうるのは各行の両端のセルだけなので、候補は 2·nu 点で済む。
    塗りは辺ごとの半平面判定を格子全体に numpy で掛ける (辺は数十本)。
    """
    nu, nv = mask.shape
    rows = np.flatnonzero(mask.any(axis=1))
    pts = []
    if rows.size:
        first = mask.argmax(axis=1)
        last = nv - 1 - mask[:, ::-1].argmax(axis=1)
        for r in rows.tolist():
            pts.append((r, int(first[r])))
            pts.append((r, int(last[r])))
    for iu, iv in extra_cells:
        if 0 <= iu < nu and 0 <= iv < nv:
            pts.append((int(iu), int(iv)))
    out = mask.copy()
    for iu, iv in pts:
        out[iu, iv] = True
    hull = _hull(pts)
    if len(hull) < 3:
        return out

    r_lo = min(p[0] for p in hull)
    r_hi = max(p[0] for p in hull)
    c_lo = min(p[1] for p in hull)
    c_hi = max(p[1] for p in hull)
    rr = np.arange(r_lo, r_hi + 1, dtype=np.float64)[:, None]
    cc = np.arange(c_lo, c_hi + 1, dtype=np.float64)[None, :]
    inside = np.ones((r_hi - r_lo + 1, c_hi - c_lo + 1), dtype=bool)
    n = len(hull)
    for i in range(n):
        a, b = hull[i], hull[(i + 1) % n]
        # 反時計回りの辺 a→b に対して左側 (cross >= 0) が内側。境界のセルも含める
        cr = (b[0] - a[0]) * (cc - a[1]) - (b[1] - a[1]) * (rr - a[0])
        inside &= cr >= -1e-9
    out[r_lo:r_hi + 1, c_lo:c_hi + 1] |= inside
    return out
