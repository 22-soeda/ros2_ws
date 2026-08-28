# -*- coding: utf-8 -*-
"""リング平面上の 2 次元 α-β 追尾。

docs/opponent_detection.pdf §9.1。予測は等速、観測は最近接クラスタの重心。

    p⁻ = p + v Δt,   r = z - p⁻,   p = p⁻ + α r,   v = v + (β/Δt) r

‖r‖ が gate を超える観測は棄却して外挿に落とす。gate = 0.35 m は「相手が
1 m/s で動いても 30 Hz の 1 フレームでは 33 mm しか動かない」ことと、
重心が胴体と腕の間で飛ぶ量を足した見積もりで、実測で詰め直す余地がある。
外挿が max_coast フレーム続いたら軌跡を捨てて未検出に戻す。

速度 v は追尾が既に持っている量なので、そのまま Opponent.velocity に載せる。
行動層で位置を数値微分し直すより遅れも雑音も小さい (§2.1 の追加提案)。
"""

import math


class AlphaBetaTracker:
    """相手 1 体ぶんの追尾。位置と速度は (前方 u, 左 v) の 2 成分。"""

    def __init__(self, alpha=0.5, beta=0.2, gate=0.35, max_coast=8):
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.gate = float(gate)
        self.max_coast = int(max_coast)
        self.reset()

    def reset(self):
        self.pos = None            # (u, v) [m]
        self.vel = (0.0, 0.0)      # (u̇, v̇) [m/s]
        self.coast = 0             # 連続で外挿したフレーム数
        self.extrapolated = False

    @property
    def active(self):
        return self.pos is not None

    def update(self, meas, dt):
        """1 フレーム進める。meas は (u, v) か、観測がなければ None。

        返り値は追尾中の (u, v) か、軌跡が無いときは None。
        """
        if dt <= 0.0:
            dt = 1e-3

        if self.pos is None:
            if meas is None:
                return None
            # 初観測。速度は 0 から始める
            self.pos = (float(meas[0]), float(meas[1]))
            self.vel = (0.0, 0.0)
            self.coast = 0
            self.extrapolated = False
            return self.pos

        pu = self.pos[0] + self.vel[0] * dt
        pv = self.pos[1] + self.vel[1] * dt

        if meas is not None:
            ru, rv = meas[0] - pu, meas[1] - pv
            if math.hypot(ru, rv) <= self.gate:
                self.pos = (pu + self.alpha * ru, pv + self.alpha * rv)
                self.vel = (self.vel[0] + self.beta * ru / dt,
                            self.vel[1] + self.beta * rv / dt)
                self.coast = 0
                self.extrapolated = False
                return self.pos
            # ゲートを外れた観測は「別の物」として棄却し、外挿を続ける

        self.pos = (pu, pv)
        self.coast += 1
        self.extrapolated = True
        if self.coast > self.max_coast:
            self.reset()
            return None
        return self.pos
