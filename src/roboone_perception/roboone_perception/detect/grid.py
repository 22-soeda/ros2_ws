# -*- coding: utf-8 -*-
"""リング平面上の占有グリッドと、その上の連結成分。

docs/opponent_detection.pdf §7。エッジ抽出と物体クラスタリングが同じグリッドを
共有するのがこの段の要点で、エッジのために別のアルゴリズムを足さない。

numpy だけで書いてある。scipy.ndimage も cv2 も使わないのは、Pi 5 側に
余計な依存を持ち込まないためと、ここが単体テストの主戦場だから。1 フレームの
セル数は 5 cm セル・前方 4 m で 90x120 = 10800 程度で、この規模なら
連結成分の union-find を Python で回しても 1〜2 ms に収まる。速度が問題に
なったら label_components() を cv2.connectedComponentsWithStats に差し替える。
差し替え先はこの 1 関数だけで済むように切ってある (§10 の 3 番目の手)。
"""

from dataclasses import dataclass

import numpy as np


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

    隣接ペアだけ numpy で作り、union-find は Python で回す。ペア数は占有セル数の
    2 倍程度で、リング 1.8 m^2 なら 1500 前後にしかならない。
    """
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

    parent = list(range(idx.size))

    def find(x):
        root = x
        while parent[root] != root:
            root = parent[root]
        while parent[x] != root:      # 経路圧縮
            parent[x], x = root, parent[x]
        return root

    for arr in pairs:
        for x, y in arr.tolist():
            rx, ry = find(x), find(y)
            if rx != ry:
                parent[max(rx, ry)] = min(rx, ry)

    roots = np.fromiter((find(i) for i in range(idx.size)), dtype=np.int32,
                        count=idx.size)
    uniq, inv = np.unique(roots, return_inverse=True)
    labels.ravel()[idx] = inv + 1
    return labels, int(uniq.size)


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
