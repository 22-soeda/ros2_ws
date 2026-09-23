# -*- coding: utf-8 -*-
"""相手の追跡と転倒判定。docs/behavior_planning.pdf §2.2 / §2.3。

検出器 (roboone_perception) も α-β 追尾を持っているのに、行動層でもう一段
持つ理由は 1 つだけである。**検出器は自分が動いていることを知らない。**
検出器の追尾はリング平面上の等速モデルで、機体が旋回すれば静止した相手が
「動いている」ように見える。観測が途切れている間の外挿を機体の運動で回すには、
自分が出した指令を知っている側でやるしかない。

したがってここでの役割分担は

    検出器 … 1 フレームの中で相手を見つけ、雑音を均す
    行動層 … 観測が来ない間、自分の動きぶんだけ相手を動かして持ちこたえる

で、観測が来ている間はほぼ検出器の値をそのまま使う（α = 0.5 で 2 フレーム）。
"""

import math

from .types import KIND_ROBOT_FALLEN, KIND_ROBOT_STANDING


def _rotate(x, y, a):
    c, s = math.cos(a), math.sin(a)
    return c * x - s * y, s * x + c * y


class OpponentTracker:
    """相手 1 体ぶんの追跡。位置は機体座標 (x 前, y 左)。

    式 (2) の予測と式 (3) の更新をそのまま実装したもの。ゲートを外れた観測は
    1 回だけ捨て、2 回続いたら乗り換える（相手が跳んだのか雑音なのかは
    1 フレームでは区別できないので、続いたかどうかで決める）。
    """

    def __init__(self, tune, robot):
        self.t = tune
        self.r = robot
        self.reset()

    def reset(self):
        self.pos = None             # (x, y) [m] 機体座標
        self.t_since_obs = float('inf')   # [s] 最後に受理した観測からの経過
        self.rejects = 0            # 連続でゲートを外れた回数
        self.last_bearing = None    # [rad] 最後に見えた方位 β_last
        self.range_at_loss = None   # [m] 見失った瞬間の ρ。死角かどうかの判定材料

    # ------------------------------------------------------------------ 状態
    @property
    def active(self):
        """軌跡を持っているか。見失い判定とは別（見失っても位置は保つ）。"""
        return self.pos is not None

    @property
    def visible(self):
        """§2.2 の「見失い」に入っていないか。T_lost 以内に観測があった。"""
        return self.pos is not None and self.t_since_obs < self.t.lost_time

    @property
    def rng(self):
        return None if self.pos is None else math.hypot(self.pos[0], self.pos[1])

    @property
    def bearing(self):
        return None if self.pos is None else math.atan2(self.pos[1], self.pos[0])

    @property
    def lost_in_blind_spot(self):
        """見失った位置が深度の死角の中だったか（§3.3「近距離の死角」）。

        これを区別する材料は見失う直前の ρ しかない。近ければ「間合いに入って
        見えなくなった」、遠ければ「本当に見失った」として扱いを分ける。
        """
        if self.range_at_loss is None:
            return False
        return self.range_at_loss < self.r.blind_range + self.t.close_margin

    # ------------------------------------------------------------------ 更新
    def step(self, meas, ego, dt):
        """1 周期進める。

        meas … 観測 (x, y) か、来ていなければ None
        ego  … 前周期に自分が出した指令 (vx, vy, wz)
        """
        if dt <= 0.0:
            dt = 1e-3
        self.t_since_obs += dt

        if self.pos is None:
            if meas is not None:
                self.pos = (float(meas[0]), float(meas[1]))
                self.t_since_obs = 0.0
                self.rejects = 0
                self.last_bearing = self.bearing
                self.range_at_loss = None
            return self.pos

        # 式 (2)。自分が (vx, vy, wz) で動くと、静止した相手はこう動いて見える
        vx, vy, wz = ego
        px = self.pos[0] - vx * dt
        py = self.pos[1] - vy * dt
        px, py = _rotate(px, py, -wz * dt)

        if meas is not None:
            dx, dy = meas[0] - px, meas[1] - py
            if math.hypot(dx, dy) <= self.t.track_gate:
                # 式 (3)
                a = self.t.track_alpha
                self.pos = (px + a * dx, py + a * dy)
                self.t_since_obs = 0.0
                self.rejects = 0
                self.last_bearing = self.bearing
                self.range_at_loss = None
                return self.pos
            self.rejects += 1
            if self.rejects >= self.t.track_gate_relax:
                # 外れ値が続いた。雑音ではなく相手が跳んだとみて乗り換える
                self.pos = (float(meas[0]), float(meas[1]))
                self.t_since_obs = 0.0
                self.rejects = 0
                self.last_bearing = self.bearing
                self.range_at_loss = None
                return self.pos

        # 観測が無い（または捨てた）周期は外挿だけ。見失った瞬間の ρ を記録する
        if self.range_at_loss is None and self.t_since_obs >= self.t.lost_time:
            self.range_at_loss = math.hypot(px, py)
        self.pos = (px, py)
        return self.pos

    def drop(self):
        """軌跡を捨てる。死角の保持時間も尽きたときに呼ぶ。

        last_bearing は残す。SEARCH がどちら向きに回るかの手掛かりになる。
        """
        self.pos = None
        self.rejects = 0
        self.range_at_loss = None


class FallenDetector:
    """相手の転倒判定。高さの分類は検出器がやり、ここは時間の門だけ持つ。

    **しきい値はこちらに持たない** (2026-09-23)。以前は「はじめ」の直後に相手の
    立位高さ H_o を測り、その比 (κ_d = 0.5 / κ_u = 0.75) で転倒と復帰を決めていた。
    比は較正が当たっているかどうかで意味が変わるので、会場で yaml の数字を見ても
    何が起きるか読めない。いまは検出器がクラスタ上端 z_top の**絶対値**で
    KIND_ROBOT_FALLEN / KIND_ROBOT_STANDING に割り、ここはその 1 バイトを
    受けるだけにしてある。高さの境界を動かすのは opponent_detector.yaml の
    match.fallen_top_max / robot_top_max で、置き場は 1 か所。

    ここに残るのは「何秒続いたら認めるか」(T_down / T_up) と、間合いの中での
    扱い (_step_close) だけ。1 フレームの分類がばたついても状態が暴れないための門で、
    高さとは別の話なので behavior.yaml に置いたままにする。
    """

    def __init__(self, tune, robot):
        self.t = tune
        self.r = robot
        self.reset()

    def reset(self):
        self.fallen = False
        self._below = 0.0          # [s] 転倒の分類が続いている時間
        self._above = 0.0
        #: 直近の周期の証拠。テストランのビューアが表示する
        self.evidence = {'low': False, 'wide': False, 'high': False,
                         'close': False}

    def step(self, kind, width, rng, dt):
        """1 周期進める。

        引数:
            kind   Opponent.kind (KIND_ROBOT_FALLEN / KIND_ROBOT_STANDING)。
                   相手が見えていない周期は None
            width  [m] クラスタの横の広がり。間合いの中の判定にだけ使う
            rng    [m] 相手までの水平距離
            dt     [s]
        """
        if kind is None:
            return self.fallen

        low = kind == KIND_ROBOT_FALLEN
        high = kind == KIND_ROBOT_STANDING
        close = rng is not None and rng < self.r.strike_range + self.t.fallen_freeze_margin
        wide = width is not None and width > self.t.fallen_width_min
        # 幅は間合いの中でしか判定に入らない。遠くで点けると「転倒の証拠が
        # 立っている」ように見えるので、効いているときだけ証拠として出す
        self.evidence = {'low': bool(low), 'wide': bool(wide and close),
                         'high': bool(high), 'close': bool(close)}
        if close:
            return self._step_close(low, wide, dt)

        self._below = self._below + dt if low else 0.0
        self._above = self._above + dt if high else 0.0

        if not self.fallen and self._below >= self.t.fallen_time:
            self.fallen = True
        elif self.fallen and self._above >= self.t.stand_time:
            self.fallen = False
        return self.fallen

    def _step_close(self, low, wide, dt):
        """間合いの中での転倒判定。文書 §2.3 から変えたところ。

        文書は ρ < ρ_s + 0.2 m で判定をまるごと凍結して直前の答えを保つ、と
        している。カメラが相手の上部しか映さなくなり、z_top が切れて低く出る
        ためで、理屈は正しい。しかしそのまま実装すると、

            ρ_s + 0.2 = 0.45 m > ρ_s = 0.25 m

        なので、**間合いに入ってから相手が倒れた場合を一度も検出できない**。
        技を当てて倒した直後がまさにこれで、規則 10.2(b)(i)（ダウン中の攻撃は
        イエローカード）が効く場面そのものを取り逃がす。

        そこで凍結を方向ごとに分けた。

        * 復帰（転倒 → 立位）は凍結したまま。上端が視野で切れるので「立っている」が
          言えない。RETREAT が ρ_r = 0.6 m まで離れれば凍結は外れ、そこで判定
          できる。離れる前に復帰を認める必要はない
        * 転倒（立位 → 転倒）は通す。ただし「上端が低い」だけでは視野の切れと
          区別が付かないので、**横に広い**ことを併せて要求する。上端が切れても
          クラスタが横に広がることはないので、これで視野由来の誤検出は落ちる

        併せて要求するぶん見落としは増えるが、見落とせば攻撃を続けるだけで、
        誤検出すれば無意味に離れる。規則の重みからは前者を避ける側に倒す。

        カメラを水平付けにしてから (2026-09-20)、視野で切れる高さは距離 ρ で
        0.365 + 0.554 ρ [m] になり、ρ_s = 0.25 m でも 0.50 m まで見える。
        match.robot_top_max = 0.60 の相手が切れるのはこの縁だけなので、
        横幅の上乗せは以前ほど効かないが、保険として残してある。
        """
        if self.fallen:
            self._above = 0.0
            return True
        self._below = self._below + dt if (low and wide) else 0.0
        self._above = 0.0
        if self._below >= self.t.fallen_time:
            self.fallen = True
        return self.fallen
