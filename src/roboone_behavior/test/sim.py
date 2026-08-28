# -*- coding: utf-8 -*-
"""行動層のテスト用の合成シーン。

roboone_perception の test/scene.py と同じ役割で、ROS を使わずに観測列を作る。
相手はリング座標系に置き、機体の推定位置から見た極座標に落として Observation を
組み立てる。検出器の癖（視野 87 deg、深度の最短距離、取りこぼし）はここで模す。

「機体の推定位置」を使うので、これは**閉ループの模擬ではない**。歩行が指令
どおりに動く前提の運動学だけで、滑りも遅れも入っていない。状態遷移の順番と
条件を固定するためのもので、実機の挙動を主張するものではない。
"""

import math

from roboone_behavior.behavior import Observation, STATUS_NO_OPPONENT, STATUS_OK

#: D435if の水平視野の半角
FOV = math.radians(43.5)
#: 深度の最短距離。これより近い相手は見えない
BLIND = 0.25
#: /motion/state のうち「歩ける」状態。behavior_node の motion_ready_states と揃える
READY = frozenset(('HOLD', 'WALK', 'MOTION', 'IDLE', 'START', 'STEP', 'STOP'))


class Scene:
    """相手 1 体と、それを見る機体。

    core.pose を「本当の位置」として使う。オドメトリの誤差を入れないので、
    リング座標系の推定はここでは常に正しい。縁まわりのテストはその前提で読む。
    """

    def __init__(self, core, opponent=(0.0, 0.0), top=0.35, width=0.20):
        self.core = core
        self.opponent = opponent
        self.top = top
        self.width = width
        self.motion_state = 'WALK'
        self.busy_left = 0          # 技の再生が残っている周期数
        self.busy_period = 20       # 技 1 回 = 1.0 s (dt=0.05)
        self.fired = []             # 出た技の名前
        self.states = []            # 通った状態の並び
        self.t = 0.0

    # ------------------------------------------------------------ 観測
    def polar(self):
        """機体座標での (ρ, β)。"""
        c = self.core.pose
        dx, dy = self.opponent[0] - c.x, self.opponent[1] - c.y
        ca, sa = math.cos(-c.yaw), math.sin(-c.yaw)
        bx, by = ca * dx - sa * dy, sa * dx + ca * dy
        return math.hypot(bx, by), math.atan2(by, bx), (bx, by)

    def observe(self, dt, autonomy=True, estop=False, drop=False):
        rng, bear, xy = self.polar()
        seen = (not drop) and abs(bear) < FOV and rng > BLIND
        obs = Observation(dt=dt, autonomy=autonomy, estop=estop,
                          motion_state=self.motion_state,
                          motion_busy=self.busy_left > 0,
                          motion_ready=self.motion_state in READY)
        obs.opponent_fresh = True       # 検出器は見えなくても publish し続ける
        obs.opponent_status = STATUS_OK if seen else STATUS_NO_OPPONENT
        if seen:
            obs.opponent_xy = xy
            obs.opponent_top = self.top
            obs.opponent_width = self.width
        return obs

    # ------------------------------------------------------------ 進める
    def step(self, dt=0.05, **kw):
        obs = self.observe(dt, **kw)
        cmd = self.core.update(obs)
        self.t += dt
        self.states.append(cmd.state)
        if cmd.motion:
            self.fired.append((round(self.t, 3), cmd.motion))
            self.busy_left = self.busy_period
        elif self.busy_left > 0:
            self.busy_left -= 1
        return cmd

    def run(self, seconds, dt=0.05, on_step=None, **kw):
        """指定した秒数ぶん回す。on_step(scene, cmd) で途中に手を入れられる。"""
        last = None
        for _ in range(int(round(seconds / dt))):
            last = self.step(dt, **kw)
            if on_step is not None:
                on_step(self, last)
        return last

    # ------------------------------------------------------------ 検査の補助
    def entered(self, state):
        return state in self.states

    def first_index(self, state):
        return self.states.index(state) if state in self.states else None
