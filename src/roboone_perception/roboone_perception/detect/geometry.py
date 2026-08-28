# -*- coding: utf-8 -*-
"""深度画像の逆投影と、リング平面座標への移し替え。

docs/opponent_detection.pdf §4 (間引きと逆投影) と §2.2 (座標系)。

カメラ座標は D435if の慣例で x 右・y 下・z 前。この座標のままでは機体が傾いた
瞬間に高さの意味が変わるので、鉛直上向き u から直交基底を作って

    h = p·u        カメラ原点を 0 とした高さ
    (u, v) = (p·e1, p·e2)   リング平面上の前方と左

に移す。方位 β = atan2(v, u) が「機体の正面から見た方位」であるためには、e1 の
出どころに注意が要る。

文書 §2.2 は e1 を**光軸**の水平成分に取っているが、俯角 θ で下を向いたカメラでは、
機体が左右にロールすると光軸自体が水平面内で振れる。ロール φ に対して光軸の
方位は atan(sinθ sinφ / cosθ) ≒ tanθ·φ ずれ、俯角 30 度なら**ロール 8 度で方位が
4.6 度動く**。歩行中のロールはこの程度は常時あるので、行動層はありもしない
方位変化を追いかけることになる（合成シーンで実測 78 mm の横ずれ）。

そこで e1 は**機体の前方軸**の水平成分から作る。カメラは機体に固定なので、
機体前方はカメラ座標で (0, -sinθ, cosθ) の定ベクトルになる（θ=0 なら光軸に一致）。
純ロールでは機体前方は水平のままなので、e1 は振れない。
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Intrinsics:
    """depth の内部パラメータ。CameraInfo からそのまま作れる。

    歪み係数は持たない。§4 のとおり桜木町の記録では brown_conrady の係数が
    5 つとも 0 で入っており、D435 系の depth は工場校正済みで出てくる。
    """

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @staticmethod
    def from_camera_info(msg):
        k = msg.k
        return Intrinsics(int(msg.width), int(msg.height),
                          float(k[0]), float(k[4]), float(k[2]), float(k[5]))


class Deprojector:
    """画素→方向ベクトルの表を作り置きして、毎フレームは深度を掛けるだけにする。

    表は (intrinsics, stride) が変わったときだけ作り直す。1 フレームあたりの
    仕事を「間引き → 有効判定 → 乗算」だけに減らすためのもので、§4 の
    「逆投影 1.5 ms」はこの形での値。
    """

    def __init__(self, intr: Intrinsics, stride: int, border_px: int = 4):
        self.intr = intr
        self.stride = int(stride)
        self.border_px = int(border_px)
        rows = np.arange(0, intr.height, self.stride)
        cols = np.arange(0, intr.width, self.stride)
        cc, rr = np.meshgrid(cols, rows)
        self._x = ((cc - intr.cx) / intr.fx).astype(np.float32)
        self._y = ((rr - intr.cy) / intr.fy).astype(np.float32)
        # 視野の縁に接する画素。ここから作ったセルは「床が切れた」ではなく
        # 「そこまでしか見えていない」なので、ring_edge では NaN にする (§7)
        self._border = ((cc < border_px) | (cc >= intr.width - border_px) |
                        (rr < border_px) | (rr >= intr.height - border_px))
        self.shape = cc.shape

    def matches(self, intr: Intrinsics, stride: int, border_px: int) -> bool:
        """この表をそのまま使い回せるか。CameraInfo が来るたびに呼ぶ。"""
        return (self.intr == intr and self.stride == int(stride) and
                self.border_px == int(border_px))

    def __call__(self, depth, depth_scale=0.001, z_min=0.15, z_max=6.0):
        """深度画像 (uint16) から点群を作る。

        返り値は (points[N,3] float32, border[N] bool)。border は「その点の
        元画素が視野の縁にあるか」。
        """
        z = depth[::self.stride, ::self.stride].astype(np.float32) * depth_scale
        # 0 は「測距失敗」で、距離 0 の点ではない。ここで落とさないと原点付近に
        # 巨大な塊ができて、リング面のヒストグラムまで汚れる。
        ok = (z > z_min) & (z < z_max)
        zv = z[ok]
        pts = np.empty((zv.size, 3), dtype=np.float32)
        pts[:, 0] = self._x[ok] * zv
        pts[:, 1] = self._y[ok] * zv
        pts[:, 2] = zv
        return pts, self._border[ok]


def forward_ref(cam_pitch_deg):
    """機体の前方軸をカメラ座標で表したもの。俯角だけで決まる定ベクトル。"""
    t = math.radians(cam_pitch_deg)
    return np.array([0.0, -math.sin(t), math.cos(t)])


def ring_basis(u, fwd=(0.0, 0.0, 1.0)):
    """鉛直 u から (e1: 前方, e2: 左) を作る。式 (1)(2)。

    e1 は基準方向 fwd の水平成分。fwd には forward_ref(俯角) を渡す
    (既定の (0,0,1) は光軸で、文書どおりの定義。上の注記を参照)。
    基準方向が u と平行だと退化するので、そのときは x 軸から作り直す
    (機体が倒れている状態で呼ばれうる)。
    """
    u = np.asarray(u, dtype=np.float64)
    u = u / np.linalg.norm(u)
    z = np.asarray(fwd, dtype=np.float64)
    e1 = z - np.dot(z, u) * u
    n = np.linalg.norm(e1)
    if n < 1e-3:
        x = np.array([1.0, 0.0, 0.0])
        e1 = x - np.dot(x, u) * u
        n = np.linalg.norm(e1)
    e1 = e1 / n
    e2 = np.cross(u, e1)
    return e1, e2


def to_plane(points, u, e1, e2):
    """点群をリング平面座標へ移す。返り値は (h, fwd, left) の 3 本の 1 次元配列。"""
    m = np.stack([np.asarray(u), np.asarray(e1), np.asarray(e2)]).astype(np.float32)
    huv = points @ m.T
    return huv[:, 0], huv[:, 1], huv[:, 2]
