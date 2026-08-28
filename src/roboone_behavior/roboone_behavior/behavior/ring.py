# -*- coding: utf-8 -*-
"""リング座標系での自己位置と、縁までの距離。

docs/behavior_planning.pdf §2.4 と §4.8。リングは既知の八角形なので、自分の
位置 (x_r, y_r) が分かれば縁までの距離は閉形式で出る。式 (5) は

    d_edge = min( a-|x|, a-|y|, (2a-c-|x|-|y|)/√2 )

だが、ここでは同じものを 8 本の半平面 n·p ≤ b として持っている。距離だけでなく
「視線方向に縁まで何 m あるか」（床の切れ目との突き合わせに要る）も同じ表現から
出せるためで、値は式 (5) と一致する。

文書から変えたところ
--------------------

**先読みの縮小に下限を付けた（§4.8）。** 文書は「先読み位置が余裕の内側に
無ければ (v_x,v_y) を一様に縮める」とだけ書いてあるが、これを字義どおりに
実装すると、既に余裕を割っている位置では EDGE の「中央へ戻る」指令まで
ゼロに潰れて、縁から動けなくなる。ここでは判定を

    先読み位置の d_edge ≥ min(d_margin, 現在地の d_edge)

にしてある。余裕の中にいる間は文書どおり、割ってしまったあとは「今より
悪化させない指令なら通す」になり、中央へ戻る成分は残る。
"""

import math

#: 半平面の法線。単位ベクトルにしてあるので b - n·p がそのまま距離になる
_S = 1.0 / math.sqrt(2.0)
_NORMALS = ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0),
            (_S, _S), (_S, -_S), (-_S, _S), (-_S, -_S))


def _bounds(match):
    """8 本の半平面の右辺 b。前 4 本が辺、後ろ 4 本が角の落とし。"""
    a = match.ring_half_width
    corner = (2.0 * a - match.ring_corner_cut) * _S
    return (a, a, a, a, corner, corner, corner, corner)


def distance_to_edge(match, x, y):
    """(x, y) から縁までの最短距離。式 (5)。外に出ていれば負。"""
    return min(b - (n[0] * x + n[1] * y) for n, b in zip(_NORMALS, _bounds(match)))


def ray_to_edge(match, x, y, theta):
    """(x, y) から方位 theta の向きに縁まで何 m あるか。

    d_cliff（床の切れ目）と突き合わせる相手はこちらで、最短距離ではない。
    凸なので、外向きの半平面との交点のうち一番近いものが答えになる。
    """
    dx, dy = math.cos(theta), math.sin(theta)
    best = float('inf')
    for n, b in zip(_NORMALS, _bounds(match)):
        nd = n[0] * dx + n[1] * dy
        if nd <= 1e-9:
            continue                     # その辺からは遠ざかる向き
        t = (b - (n[0] * x + n[1] * y)) / nd
        if t < best:
            best = t
    return best


class RingPose:
    """リング座標系での自分の位置と向き、および最後の補正からの歩行距離 s。

    位置は「はじめ」の時点の自陣コーナーで初期化し、/odom があればそれで、
    無ければ自分が出した指令の積分で進める。どちらにしても滑りで狂うので、
    最後に補正してからの距離 s に比例して縁の余裕を増やす（式 (6)）。
    """

    def __init__(self, match, robot, tune):
        self.m = match
        self.r = robot
        self.t = tune
        self.reset()

    def reset(self):
        self.x = self.r.start_x
        self.y = self.r.start_y
        self.yaw = self.r.start_yaw
        self.s = 0.0               # [m] 最後に補正してからの歩行距離
        self._odom_prev = None     # 前周期の /odom。差分だけを使う
        self.from_odom = False     # 直近の 1 周期を /odom で進めたか

    # ------------------------------------------------------------------ 更新
    def step(self, odom, cmd, dt):
        """1 周期進める。odom があれば差分を使い、無ければ指令を積分する。

        /odom の絶対値ではなく差分を使うのは、歩行ノードのオドメトリの原点が
        リング座標系の原点と一致しないためである。原点合わせは「はじめ」の
        時点の自陣コーナー（robot.start_*）が受け持つ。
        """
        if odom is not None:
            if self._odom_prev is not None:
                ox, oy, oyaw = odom
                px, py, pyaw = self._odom_prev
                dx, dy = ox - px, oy - py
                self.x += dx
                self.y += dy
                self.yaw = _wrap(self.yaw + _wrap(oyaw - pyaw))
                self.s += math.hypot(dx, dy)
            self._odom_prev = tuple(odom)
            self.from_odom = True
            return

        self._odom_prev = None
        self.from_odom = False
        vx, vy, wz = cmd
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx = (c * vx - s * vy) * dt
        dy = (s * vx + c * vy) * dt
        self.x += dx
        self.y += dy
        self.yaw = _wrap(self.yaw + wz * dt)
        self.s += math.hypot(dx, dy)

    # ------------------------------------------------------------------ 縁
    @property
    def d_edge(self):
        return distance_to_edge(self.m, self.x, self.y)

    @property
    def d_margin(self):
        """式 (6)。最後に補正してからの歩行距離ぶんだけ余裕を増やす。"""
        return self.t.edge_margin0 + self.t.edge_margin_slip * self.s

    def to_center(self):
        """中央へ向かう単位ベクトル ê_c を機体座標で返す。中央にいれば (0,0)。"""
        n = math.hypot(self.x, self.y)
        if n < 1e-6:
            return (0.0, 0.0)
        # R_z(-ψ)·(-r)/|r|
        ux, uy = -self.x / n, -self.y / n
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        return (c * ux - s * uy, s * ux + c * uy)

    def correct_with_cliff(self, d_cliff, bearing=0.0):
        """見えている床の切れ目でリング座標系を 1 次元だけ補正する（§2.4）。

        視線方向に縁まで何 m あるかの予測と観測の差だけ、自分を視線方向へ
        ずらす。差が cliff_fix_gate を超えたら別の縁か誤検出として捨てる
        （リングの外の床や、相手の陰で打ち切られた値が来うる）。

        返り値は補正できたか。補正したら s を 0 に戻す。
        """
        if d_cliff is None or not math.isfinite(d_cliff):
            return False
        theta = self.yaw + bearing
        pred = ray_to_edge(self.m, self.x, self.y, theta)
        if not math.isfinite(pred):
            return False
        delta = pred - d_cliff
        if abs(delta) > self.t.cliff_fix_gate:
            return False
        self.x += delta * math.cos(theta)
        self.y += delta * math.sin(theta)
        self.s = 0.0
        return True

    def clamp_lookahead(self, vx, vy, horizon):
        """先読みして縁へ向かう成分だけ削る（§4.8）。冒頭の注記も見ること。

        返り値は掛けた倍率 k ∈ [0, 1]。呼び出し側で (vx, vy) に掛ける。
        """
        if horizon <= 0.0 or (abs(vx) < 1e-9 and abs(vy) < 1e-9):
            return 1.0
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ux = (c * vx - s * vy) * horizon
        uy = (s * vx + c * vy) * horizon
        need = min(self.d_margin, self.d_edge)

        def ok(k):
            return distance_to_edge(self.m, self.x + k * ux, self.y + k * uy) >= need

        if ok(1.0):
            return 1.0
        # d_edge は半平面の min なので、この直線上では凹関数になる。
        # よって {k : ok(k)} は 0 を含む区間で、二分探索で端が出る
        lo, hi = 0.0, 1.0
        for _ in range(24):
            mid = 0.5 * (lo + hi)
            if ok(mid):
                lo = mid
            else:
                hi = mid
        return lo


def _wrap(a):
    """角度を (-π, π] に畳む。"""
    return math.atan2(math.sin(a), math.cos(a))


def forward_cliff(cliff, half_fov, bearing=0.0, span=None):
    """d_cliff(θ) の配列から、ある方位まわりの最小値を取り出す。

    ビン k の中心方位は -half + (k+0.5)·2·half/len（検出器側 _edge_msg の約束）。
    NaN は「縁が無い」ではなく「見ていない」なので、平均ではなく
    「見えているビンの最小値」を取る。1 つも見えていなければ None。
    """
    if not cliff:
        return None
    n = len(cliff)
    if span is None:
        span = 2.0 * half_fov / n * 1.5     # 前方 3 ビンぶん
    step = 2.0 * half_fov / n
    best = None
    for k, v in enumerate(cliff):
        if v is None or not math.isfinite(v):
            continue
        theta = -half_fov + (k + 0.5) * step
        if abs(_wrap(theta - bearing)) > span:
            continue
        if best is None or v < best:
            best = float(v)
    return best
