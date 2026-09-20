# -*- coding: utf-8 -*-
"""グリッドを自機から見た極座標 (方位ビン, 距離) で引くための索引。

影と「リングの外」は、どちらも**自機から見た視線に沿って**決まる量なので、直交の
グリッドのままでは書けない。ここに 2 つだけ置く (docs/相手機の認識.md §3, §6)。

* **影** … 面より上の物の背後で、床が写らなかったセル。近い相手は見えている床を
  左右 2 つに分断するので、影をまたいでつながないと片側しかリングとして拾えない。
* **確かに外** … その方位で床が「自分で終わっている」(終わりに物が立っていない) とき、
  その先にある面より上のセル。床が物に隠されて終わった方位や、床が 1 画素も写って
  いない方位 (足元の死角) は「分からない」であって外ではない。視野の端に立つ相手を
  外の物と取り違えないための区別。
"""

import math

import numpy as np


class PolarIndex:
    """セルごとの方位ビンと距離。GridSpec ごとに 1 回作って使い回す。"""

    def __init__(self, spec, bin_deg=4.0, half_deg=64.0):
        iu = np.arange(spec.nu)[:, None]
        iv = np.arange(spec.nv)[None, :]
        fwd, left = spec.centers(iu, iv)
        fwd = np.broadcast_to(fwd, spec.shape).astype(np.float64)
        left = np.broadcast_to(left, spec.shape).astype(np.float64)
        self.cell = float(spec.cell)
        self.rng = np.hypot(fwd, left)
        ang = np.degrees(np.arctan2(left, fwd))
        self.nbins = int(round(2 * half_deg / bin_deg))
        b = np.floor((ang + half_deg) / bin_deg).astype(np.int64)
        ok = (fwd > 0) & (b >= 0) & (b < self.nbins)
        self.bin = np.where(ok, b, -1)

    def _per_bin(self, mask, op, init):
        out = np.full(self.nbins, init, dtype=np.float64)
        sel = mask & (self.bin >= 0)
        if sel.any():
            op.at(out, self.bin[sel], self.rng[sel])
        return out

    def nearest(self, mask):
        """方位ビンごとの、mask のセルまでの最短距離。無ければ inf。"""
        return self._per_bin(mask, np.minimum, np.inf)

    def farthest(self, mask):
        """方位ビンごとの、mask のセルまでの最長距離。無ければ -inf。"""
        return self._per_bin(mask, np.maximum, -np.inf)

    def _lookup(self, per_bin, fill):
        out = np.full(self.bin.shape, fill, dtype=np.float64)
        ok = self.bin >= 0
        out[ok] = per_bin[self.bin[ok]]
        return out

    def shadow(self, above, length):
        """影のセル。above のセルの背後 length [m] まで。above 自身は含まない。

        近距離のセルは 1 つで複数の方位ビンにまたがるので、隣のビンへも影を広げる。
        """
        near = self.nearest(above)
        spread = np.minimum(near, np.minimum(np.roll(near, 1), np.roll(near, -1)))
        spread[0] = min(near[0], near[1]) if self.nbins > 1 else near[0]
        spread[-1] = min(near[-1], near[-2]) if self.nbins > 1 else near[-1]
        r0 = self._lookup(spread, np.inf)
        return (self.rng > r0) & (self.rng <= r0 + float(length)) & ~above

    def beyond_floor_end(self, above, floor):
        """確かにリングの外のセル。above のうち「床が自分で終わった先」にあるもの。"""
        end = self.farthest(floor)                       # その方位で床が見えた最遠
        r_end = self._lookup(end, -np.inf)
        # 床の終わりに物が立っている方位。そこでは床は隠されて終わっただけ
        at_end = above & np.isfinite(r_end) & (
            np.abs(self.rng - r_end) <= 2.0 * self.cell)
        hidden = np.zeros(self.nbins, dtype=bool)
        if at_end.any():
            hidden[self.bin[at_end & (self.bin >= 0)]] = True
            hidden = hidden | np.roll(hidden, 1) | np.roll(hidden, -1)
        hid = np.zeros(self.bin.shape, dtype=bool)
        ok = self.bin >= 0
        hid[ok] = hidden[self.bin[ok]]
        return (above & np.isfinite(r_end) & ~hid
                & (self.rng > r_end + 2.0 * self.cell))


def fov_half_deg(intr):
    """水平画角の半分 [deg]。デバッグ表示用。"""
    return math.degrees(math.atan(0.5 * intr.width / intr.fx))
