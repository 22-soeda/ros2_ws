# -*- coding: utf-8 -*-
r"""テスト用の合成シーン。会場を模した深度画像を作る。

実機の bag は手元 (C:\...\realsense_test) にあってこのリポジトリには無いので、
単体テストは docs/opponent_detection.pdf が記述している会場の幾何をそのまま
描いて作る。狙いは「文書が挙げた失敗の形を再現できること」で、具体的には

  * リング面と場外の床が 34 cm の段差で 2 つの峰を作る (§6.1)
  * 場外の什器がリング上の相手と同じかそれより高い位置にある (§1.2, §8.1)
  * 視野より広いリングでは左右の「境界」が視野の縁になる (§7)

を作れるようにしてある。ここで通るからといって実機で通る保証はない。
実機の検証は bag と Pi 5 実測でやる (§12)。

座標系は世界 (X 前・Y 左・Z 上、リング面を Z=0)。カメラは D435if の慣例で
x 右・y 下・z 前。深度画像の値は光軸方向の z で、距離ではない。
"""

import math

import numpy as np


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def mount_matrix(pitch_deg):
    """俯角 pitch_deg で前を向いたカメラの、カメラ→世界の回転。

    カメラ軸を世界で書くと x_cam=(0,-1,0) (右), y_cam=(-sinθ,0,-cosθ) (下),
    z_cam=(cosθ,0,-sinθ) (前で下向き)。列に並べたものが回転行列になる。
    このとき世界の上向き (0,0,1) はカメラ座標で (0,-cosθ,-sinθ) で、
    BodyParams.up_from_mount と一致する。
    """
    t = math.radians(pitch_deg)
    return np.array([
        [0.0, -math.sin(t), math.cos(t)],
        [-1.0, 0.0, 0.0],
        [0.0, -math.cos(t), -math.sin(t)],
    ])


class Box:
    """世界座標の直方体。相手にも場外の什器にも使う。"""

    def __init__(self, cx, cy, w, d, z0, z1):
        self.lo = np.array([cx - d / 2, cy - w / 2, z0])
        self.hi = np.array([cx + d / 2, cy + w / 2, z1])


class Scene:
    """リング + 場外の床 + 直方体いくつか。"""

    def __init__(self, ring_half=1.8, corner_cut=0.9, drop=0.34, boxes=(),
                 center=(0.0, 0.0)):
        self.ring_half = ring_half
        self.corner_cut = corner_cut
        self.drop = drop
        self.boxes = list(boxes)
        #: リング中心の位置。カメラは常に世界原点にあるので、機体をリングの
        #: どこに置くかはこれで表す (縁の見え方を試すのに使う)
        self.center = center

    def on_ring(self, x, y):
        """第 44 回規則の八角形 (360cm 角の四隅を 90cm 落とす) の内側か。"""
        a, c = self.ring_half, self.corner_cut
        x = x - self.center[0]
        y = y - self.center[1]
        return ((np.abs(x) <= a) & (np.abs(y) <= a)
                & (np.abs(x) + np.abs(y) <= 2 * a - c))


def render(scene, intr, cam_height=0.35, pitch_deg=30.0, roll_deg=0.0,
           yaw_deg=0.0, noise=0.0, hole_rate=0.0, seed=0, z_max=8.0):
    """深度画像 (uint16, mm) を作る。

    roll_deg / yaw_deg は機体の傾き・向き。姿勢推定を試すときに使う。
    noise は距離の 2 乗に比例する深度雑音の係数 (σ = noise · z² [m])。
    D435 系の実測はおよそ 0.1〜0.2% z² なので noise=0.002 あたりが目安。
    """
    rng = np.random.default_rng(seed)
    r_body = rot_z(math.radians(yaw_deg)) @ rot_x(math.radians(roll_deg))
    rot = r_body @ mount_matrix(pitch_deg)
    cam = np.array([0.0, 0.0, cam_height])

    cols = np.arange(intr.width)
    rows = np.arange(intr.height)
    cc, rr = np.meshgrid(cols, rows)
    d = np.stack([(cc - intr.cx) / intr.fx,
                  (rr - intr.cy) / intr.fy,
                  np.ones_like(cc, dtype=float)], axis=-1)
    w = d @ rot.T                      # 世界での方向。パラメータ t は光軸方向の z
    best = np.full(cc.shape, np.inf)

    def hit_plane(z_plane, region=None):
        wz = w[..., 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (z_plane - cam[2]) / wz
        ok = np.isfinite(t) & (t > 0)
        x = cam[0] + t * w[..., 0]
        y = cam[1] + t * w[..., 1]
        if region is not None:
            ok &= region(x, y)
        np.minimum(best, np.where(ok, t, np.inf), out=best)

    hit_plane(0.0, scene.on_ring)                                  # リング面
    hit_plane(-scene.drop, lambda x, y: ~scene.on_ring(x, y))      # 場外の床

    for b in scene.boxes:                                          # 直方体 (スラブ法)
        t_lo = np.full(cc.shape, -np.inf)
        t_hi = np.full(cc.shape, np.inf)
        for ax in range(3):
            wi = w[..., ax]
            near = np.where(np.abs(wi) > 1e-12,
                            (b.lo[ax] - cam[ax]) / np.where(wi == 0, 1, wi),
                            -np.inf)
            far = np.where(np.abs(wi) > 1e-12,
                           (b.hi[ax] - cam[ax]) / np.where(wi == 0, 1, wi),
                           np.inf)
            lo, hi = np.minimum(near, far), np.maximum(near, far)
            inside = (cam[ax] >= b.lo[ax]) & (cam[ax] <= b.hi[ax])
            lo = np.where(np.abs(wi) > 1e-12, lo, np.where(inside, -np.inf, np.inf))
            hi = np.where(np.abs(wi) > 1e-12, hi, np.where(inside, np.inf, -np.inf))
            t_lo = np.maximum(t_lo, lo)
            t_hi = np.minimum(t_hi, hi)
        ok = (t_lo <= t_hi) & (t_hi > 0)
        t = np.where(t_lo > 0, t_lo, t_hi)
        np.minimum(best, np.where(ok, t, np.inf), out=best)

    z = np.where(np.isfinite(best) & (best < z_max), best, 0.0)
    if noise > 0:
        z = np.where(z > 0, z + rng.normal(0.0, noise, z.shape) * z * z, 0.0)
    if hole_rate > 0:
        z = np.where(rng.random(z.shape) < hole_rate, 0.0, z)
    return np.clip(z * 1000.0, 0, 65535).astype(np.uint16)
