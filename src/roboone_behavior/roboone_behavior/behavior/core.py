# -*- coding: utf-8 -*-
"""行動の本体。docs/behavior_planning.pdf §3 と §4。

ROS に依存しない。時計も乱数も持たず、Observation を順に流せば決定的に同じ
指令列が出る（§6.2）。試合の bag から Observation を作り直せば、そのとき
なぜそう動いたかを机上で再現できる。

1 周期の順序は §6.2 のとおり:

    観測の整形 → 割り込み状態の判定 → 通常状態の選択 → 指令の計算
    → 縁の事前確認 → 小移動の重ね合わせ → 飽和

文書から変えたところ（理由つき）
--------------------------------

**1. 「はじめ」の合図が /match/cmd (String) ではなく /autonomy (Bool)。**
teleop 側が既に /autonomy で実装されている（roboone_teleop の docstring）。
規則 5.1.2 の無線始動・停止機構としては同じもので、hold と stop の区別が
無いだけである。行動層から見ると WAIT に落ちるのは同じなので区別は要らない。
脱力 (/estop) も同じく WAIT に落とす。

**2. しゃがみ後の「3 歩」を時間で数える。** 規則 10.2(l) は歩数で書いてあるが、
行動層には歩の境界が見えない（/motion/state は状態が変わったときしか来ない）。
並進指令を出していた時間を T_step で割って歩数の代わりにしている。歩行が
遅れる方向に効くので、規則に対しては安全側に外れる。

**3. RETREAT は相手の軌跡を持っている間だけ。** 文書は転倒判定だけを条件に
しているが、相手を完全に見失うと「離れる向き」も「復帰したか」も決められない。
軌跡が切れたら転倒の判定ごと捨てて SEARCH に戻る。

**4. 近距離の死角から抜けるときの後退を、しゃがみ後の後退と同じ口にした。**
どちらも「少し下がってやり直す」で、同時に起きることはない。
"""

import math

from .keepalive import KeepAlive
from .params import BehaviorParams
from .ring import forward_cliff, RingPose
from .tracking import FallenDetector, OpponentTracker
from .types import (APPROACH, Command, EDGE, ENGAGE, INTERRUPTS, Observation,
                    PRIORITY, RETREAT, SEARCH, SELF_DOWN, STATES,
                    STATUS_ATTITUDE_STALE, STATUS_OK, STATUS_RING_LOST, WAIT)


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _sgn(v):
    return -1.0 if v < 0.0 else 1.0


class BehaviorCore:
    """行動層の状態機械。update(obs) を 20 Hz で回す。"""

    def __init__(self, params=None):
        self.p = params or BehaviorParams()
        m, r, t = self.p.match, self.p.robot, self.p.tune
        self.tracker = OpponentTracker(t, r)
        self.fall = FallenDetector(t, r)
        self.pose = RingPose(m, r, t)
        self.keepalive = KeepAlive(m, r, t)
        self.ever_started = False
        self.reset()

    def reset(self):
        """試合の頭に戻す。「はじめ」で呼ぶ。"""
        self.state = WAIT
        self.reason = '起動'
        self.t_in_state = 0.0
        self.last_cmd = (0.0, 0.0, 0.0)
        self.tracker.reset()
        self.fall.reset()
        self.keepalive.reset()
        self.backoff = 0.0          # [m] 下がりたい残り距離（死角 / しゃがみ後）
        self.retreat_travel = 0.0   # [m] この RETREAT で下がった距離
        self.walk_time = 0.0        # [s] 並進指令を出していた累計。歩数の代わり
        self.technique = None       # この ENGAGE で出した技。出す前は None
        self.last_technique = None  # 直前に出し終えた技。しゃがみの縛りに使う
        self.technique_started = False   # motion が再生を始めたのを見たか
        self.technique_done = True       # 出した技が終わったか
        self.t_since_fire = 0.0     # [s] 技を渡してからの経過
        self.motion_busy = False    # /motion/state の技再生中フラグの前周期値
        self.next_technique = 0     # 技を順に出すための添字

    # ================================================================== 1 周期
    def update(self, obs=None):
        """1 周期。Observation を受けて Command を返す。"""
        obs = obs or Observation()
        dt = max(float(obs.dt), 1e-3)
        p = self.p
        dbg = {}

        # ---------------------------------------------------------- 指令権
        # /autonomy が false か脱力中は WAIT。歩行指令は一切作らない（規則 10.2(k)）
        if not obs.autonomy or obs.estop:
            self._enter(WAIT, '脱力' if obs.estop else '/autonomy false')
            self.t_in_state += dt
            self.last_cmd = (0.0, 0.0, 0.0)
            return self._command(0.0, 0.0, 0.0, None, dbg)

        if self.state == WAIT:
            self._on_start()

        # ------------------------------------------------- 観測の整形 (§2)
        # 検出器が外挿しているフレームは「新しい観測」ではない。取り込むと
        # 見失いのタイマ T_lost が戻ってしまい、いつまでも古い位置を信じる。
        # 外挿するならこちらでやるほうが正確でもある（tracking.py 冒頭）
        fresh = (obs.opponent_fresh and obs.opponent_status == STATUS_OK
                 and not obs.opponent_extrapolated)
        meas = obs.opponent_xy if (fresh and obs.opponent_xy is not None) else None
        self.tracker.step(meas, self.last_cmd, dt)
        self._expire_track()

        rng = self.tracker.rng
        self.fall.step(obs.opponent_top if fresh else None,
                       obs.opponent_width if fresh else None, rng, dt)
        fallen = self.fall.fallen and self.tracker.active

        self.pose.step(obs.odom, self.last_cmd, dt)
        d_cliff = self._forward_cliff(obs)
        if d_cliff is not None:
            self.pose.correct_with_cliff(d_cliff)

        self._track_technique(obs, dt)

        # ------------------------------------------------- 状態の選択 (§3.1)
        want, why = self._select(obs, fallen, d_cliff)
        self.t_in_state += dt
        if want != self.state:
            if want in INTERRUPTS or PRIORITY[want] < PRIORITY[self.state] \
                    or self.t_in_state >= p.tune.dwell:
                self._enter(want, why)
            # T_dwell に満たない格下げは見送る。閾値の周りで往復させないため

        # ------------------------------------------------- 指令の計算 (§4)
        vx, vy, wz, motion = self._act(obs, d_cliff, dbg)

        # 技の再生中は歩行指令を出さない（§3.5）。小移動のタイマは止めない
        if obs.motion_busy:
            vx = vy = wz = 0.0
            self.keepalive.hold(dt)
        elif self.state in (WAIT, SELF_DOWN):
            self.keepalive.hold(dt)
        else:
            # ------------------------------------- 縁の事前確認 (§4.8)
            k = self.pose.clamp_lookahead(vx, vy, p.tune.lookahead)
            vx, vy = vx * k, vy * k
            dbg['edge_scale'] = k
            # ------------------------------------- 小移動の重ね合わせ (§4.9)
            vx, vy = self.keepalive.overlay(vx, vy, self.pose, dt)

        vx = _clamp(vx, -p.robot.v_max, p.robot.v_max)
        vy = _clamp(vy, -p.robot.v_y_max, p.robot.v_y_max)
        wz = _clamp(wz, -p.robot.w_max, p.robot.w_max)

        if math.hypot(vx, vy) > p.tune.move_eps:
            self.walk_time += dt

        self.last_cmd = (vx, vy, wz)
        self._fill_debug(dbg, d_cliff)
        return self._command(vx, vy, wz, motion, dbg)

    # ============================================================ 観測の整形
    def _on_start(self):
        """「はじめ」の瞬間。試合の頭かどうかで位置の扱いを変える。

        初回は自陣コーナーで初期化する。2 回目以降（「待て」から戻ったとき）は
        機体が動かされている可能性はあるが、コーナーに戻すよりは直前の推定の
        ほうが近い。ただし信用は落ちているので、余裕を最大まで開けた状態
        （s を大きく）から再開する。前方の切れ目が見えれば式 (6) の s は
        そこで 0 に戻る。
        """
        first = not self.ever_started
        self.reset()
        self.ever_started = True
        if first:
            self.pose.reset()
        else:
            self.pose.s = self.p.match.ring_half_width
        self._enter(SEARCH, '「はじめ」')

    def _expire_track(self):
        """見失いが続いた軌跡を捨てる（§3.3 の近距離の死角を含む）。"""
        tr = self.tracker
        if not tr.active or tr.visible:
            return
        hold = self.p.tune.close_hold_time if tr.lost_in_blind_spot else 0.0
        if tr.t_since_obs <= self.p.tune.lost_time + hold:
            return
        if tr.lost_in_blind_spot:
            # 近すぎて見えないまま T_close 過ぎた。下がって取り直す
            self.backoff = max(self.backoff, self.p.tune.close_back_off)
        tr.drop()
        # 相手が分からなくなったら転倒の判定も持ち越さない（冒頭の変更 3）
        self.fall.fallen = False

    def _forward_cliff(self, obs):
        """前方の d_cliff。見えているビンの最小値を取る。

        文書 §2.4 は「相手の陰では d_cliff が相手までの距離で打ち切られる」ので
        行動層で切り分けろ、としているが、検出器 (roboone_perception) が既に
        遮蔽の影を NaN にして出している。ここで距離を見比べて捨て直すと、
        相手が本物の縁の近くに立っているときに「影に見えるから」と本物の縁を
        捨てることになり、危ない側に外れる。影の切り分けは検出器に任せる。

        NaN は「見ていない」であって「縁が無い」ではない。NaN が続く方位を
        「縁が近い」と読んではいけないので、平均ではなく見えているビンの最小値
        だけを見る（forward_cliff の中）。
        """
        if obs.cliff is None:
            return None
        span = 2.0 * obs.cliff_half_fov / max(len(obs.cliff), 1) * 1.5
        return forward_cliff(obs.cliff, obs.cliff_half_fov, 0.0, span)

    def _track_technique(self, obs, dt):
        """渡した技が始まったか・終わったかを /motion/state の立ち上がりで追う。

        motion が技を受け取らなかった場合（名前が config に無い、再生できない）
        に ENGAGE から出られなくなるのを避けるため、再生が始まらないまま
        technique_timeout 過ぎたら「終わった」ことにして先へ進む。
        """
        busy = bool(obs.motion_busy)
        if self.technique is not None and not self.technique_done:
            self.t_since_fire += dt
            if busy:
                self.technique_started = True
            elif self.technique_started:
                self._finish_technique(self.technique)
            elif self.t_since_fire >= self.p.tune.technique_timeout:
                self._finish_technique(None)
        self.motion_busy = busy

    def _finish_technique(self, name):
        """技が終わった周期に 1 回だけ通る。"""
        self.technique_done = True
        self.last_technique = name
        if name in self._crouch_set():
            # 規則 10.2(l)。3 歩以上歩いてからでないと次の技を出せない。
            # 歩数を数え直すために、まず下がって間合いを開ける
            self.walk_time = 0.0
            self.backoff = max(self.backoff, self.p.tune.close_back_off)

    def _crouch_set(self):
        return set(self.p.robot.crouch_techniques or ())

    # ============================================================ 状態の選択
    def _select(self, obs, fallen, d_cliff):
        """§3.1 の優先順位で 1 つ選ぶ。返り値は (状態, 理由)。"""
        p = self.p
        tr = self.tracker

        # 2. SELF_DOWN — motion が歩ける状態にない。転倒（FALL）の起き上がりも、
        # 脱力（RELAX）からのトルク投入（ARMING）も motion の仕事で、行動層は
        # 待つだけである。ここで歩行指令を出さないことには意味が 2 つあって、
        # 機体が動かないのに指令だけ出すのを避けるのと、リング座標系の推定を
        # 「歩いたつもり」で進めないことによる（推定は自分の指令の積分なので）
        if not obs.motion_ready or obs.motion_state in ('FALL', 'ESTOP'):
            return SELF_DOWN, 'motion=%s（歩ける状態にない）' % obs.motion_state

        # 3. EDGE — 縁の余裕を割った。解除は d_1 まで戻ってから（ヒステリシス）
        margin = self.pose.d_margin
        d_edge = self.pose.d_edge
        if self.state == EDGE:
            if d_edge <= p.tune.edge_release:
                return EDGE, '中央へ戻り中 d_edge=%.2f' % d_edge
        else:
            if d_edge < margin:
                return EDGE, 'd_edge=%.2f < 余裕 %.2f' % (d_edge, margin)
            if d_cliff is not None and d_cliff < margin:
                return EDGE, '前方の切れ目 %.2f < 余裕 %.2f' % (d_cliff, margin)

        # 4. RETREAT — 相手が転倒中。規則 10.2(b)(i)
        if fallen:
            return RETREAT, '相手が転倒 (H_o=%.2f)' % self.fall.h_stand

        # 5. ENGAGE — 技を出している途中は、相手を見失っても抜けない。
        # 技の再生中は相手が視野の死角に入るのが普通で、そこで抜けると
        # 再生中の技を残したまま別の状態の指令を重ねることになる
        if self.state == ENGAGE and not self.technique_done:
            return ENGAGE, ('技 %s を再生中' % self.technique
                            if self.technique else '技を選ぶ')

        if not tr.active:
            return SEARCH, '相手が見えない'

        beta, rng = tr.bearing, tr.rng

        if (self.state != ENGAGE
                and rng <= p.robot.strike_range + p.tune.range_eps
                and abs(beta) < p.tune.bearing_in and self._may_strike()):
            return ENGAGE, 'ρ=%.2f β=%.1fdeg' % (rng, math.degrees(beta))

        # 6. APPROACH — 見えていて正面。既に APPROACH なら β_out まで許す
        limit = p.tune.bearing_out if self.state == APPROACH else p.tune.bearing_in
        if tr.visible and abs(beta) < limit:
            return APPROACH, 'ρ=%.2f β=%.1fdeg' % (rng, math.degrees(beta))
        if not tr.visible and tr.lost_in_blind_spot:
            # 近距離の死角。保持している間は間合いを維持する（§3.3）
            return APPROACH, '死角で保持中 (%.1fs)' % tr.t_since_obs

        # 7. SEARCH
        return SEARCH, ('視野の端 β=%.1fdeg' % math.degrees(beta)
                        if tr.visible else '見失い (%.1fs)' % tr.t_since_obs)

    def _may_strike(self):
        """しゃがみを含む技のあとの 3 歩の縛り（規則 10.2(l)）。"""
        if self.last_technique not in self._crouch_set():
            return True
        steps = self.walk_time / max(self.p.robot.step_period, 1e-3)
        return steps >= self.p.match.crouch_steps

    def _enter(self, state, reason):
        if state == self.state:
            self.reason = reason
            return
        self.state = state
        self.reason = reason
        self.t_in_state = 0.0
        if state == RETREAT:
            self.retreat_travel = 0.0
        if state == ENGAGE:
            self.technique = None      # 入った周期に選び直す
            self.technique_started = False
            self.technique_done = False
            self.t_since_fire = 0.0

    # ============================================================ 指令の計算
    def _act(self, obs, d_cliff, dbg):
        """状態ごとの (vx, vy, wz, 技名)。"""
        s = self.state
        if s == WAIT or s == SELF_DOWN:
            return 0.0, 0.0, 0.0, None
        if s == EDGE:
            return self._act_edge()
        if s == RETREAT:
            return self._act_retreat(obs, dbg)
        if s == ENGAGE:
            return self._act_engage(obs)
        if s == APPROACH:
            return self._act_approach(obs, d_cliff)
        return self._act_search(obs)

    def _servo(self, beta):
        """式 (7) の方位サーボ。相手を正面に保つ。"""
        w = self.p.robot.w_max
        return _clamp(self.p.tune.k_bearing * beta, -w, w)

    def _consume_backoff(self, dt_speed):
        """下がりたい距離を 1 周期ぶん減らす。返り値は後退速度（負）。"""
        v = self.p.tune.edge_speed
        self.backoff = max(0.0, self.backoff - v * dt_speed)
        return -v

    def _act_search(self, obs):
        """§4.2。見えていなければ旋回、視野の端にいれば方位サーボで寄せる。"""
        p = self.p
        tr = self.tracker
        dt = max(float(obs.dt), 1e-3)

        if self.backoff > 0.0:
            # 死角 / しゃがみ後の後退。向きは変えずにまっすぐ下がる
            return self._consume_backoff(dt), 0.0, 0.0, None

        if tr.visible:
            return 0.0, 0.0, self._servo(tr.bearing), None

        # 回る向きは最後に見えた側。それも無ければリング中央がある側を先に掃く
        if tr.last_bearing is not None:
            sign = _sgn(tr.last_bearing)
        else:
            ec = self.pose.to_center()
            sign = _sgn(math.atan2(ec[1], ec[0])) if (ec[0] or ec[1]) else 1.0
        w = p.tune.search_omega
        if obs.opponent_status in (STATUS_ATTITUDE_STALE, STATUS_RING_LOST):
            # 姿勢がジャイロ任せに落ちている。速く回すとヨーの誤差が乗る
            w *= p.tune.search_degraded_scale
        return 0.0, 0.0, sign * w, None

    def _act_approach(self, obs, d_cliff):
        """§4.3。相手の ρ_s 手前まで直線で歩く。"""
        p = self.p
        tr = self.tracker
        dt = max(float(obs.dt), 1e-3)
        beta = tr.bearing if tr.bearing is not None else 0.0
        rng = tr.rng if tr.rng is not None else 0.0
        wz = self._servo(beta)

        if self.backoff > 0.0:
            return self._consume_backoff(dt), 0.0, wz, None

        # 式 (9)。向きが合っていないうちは進まない
        vx = _clamp(p.tune.k_range * (rng - p.robot.strike_range),
                    0.0, p.robot.v_max) * max(0.0, math.cos(beta))

        # 式 (10)。前進は見えている床の上に限る
        if d_cliff is not None:
            vx = min(vx, p.tune.k_range * (d_cliff - self.pose.d_margin))
        return max(0.0, vx), 0.0, wz, None

    def _act_engage(self, obs):
        """§4.5。技を 1 回渡し、再生中は歩行指令を出さない。"""
        if obs.motion_busy or self.technique is not None:
            return 0.0, 0.0, 0.0, None
        names = list(self.p.robot.techniques or ())
        if not names:
            # 技がまだ 1 つも無い。ENGAGE に入っても出すものが無いので、
            # 間合いを保ったまま待つ（歩行だけの試走で使う形）
            self.technique_done = True
            return 0.0, 0.0, 0.0, None
        self.technique = names[self.next_technique % len(names)]
        self.next_technique += 1
        self.technique_started = False
        self.technique_done = False
        self.t_since_fire = 0.0
        return 0.0, 0.0, 0.0, self.technique

    def _act_retreat(self, obs, dbg):
        """§4.6。相手の反対向きと中央向きを混ぜて離れ、向きは相手に保つ。"""
        p = self.p
        tr = self.tracker
        dt = max(float(obs.dt), 1e-3)
        rng = tr.rng or 0.0
        wz = self._servo(tr.bearing or 0.0)

        # 式 (11) の λ。縁に近いほど中央へ曲げる
        d0, d1 = self.pose.d_margin, p.tune.edge_release
        lam = _clamp((d1 - self.pose.d_edge) / max(d1 - d0, 1e-3), 0.0, 1.0)
        dbg['lambda'] = lam

        done = (rng >= p.match.retreat_range
                or self.retreat_travel >= p.match.retreat_max_travel)
        if done:
            # 離れ終わった。向きだけ相手に保って復帰を待つ（小移動は §4.9 が入れる）
            return 0.0, 0.0, wz, None

        ex, ey = (tr.pos[0] / rng, tr.pos[1] / rng) if rng > 1e-6 else (1.0, 0.0)
        cx, cy = self.pose.to_center()
        ux, uy = -ex + lam * cx, -ey + lam * cy
        n = math.hypot(ux, uy)
        if n < 1e-6:
            ux, uy, n = -ex, -ey, 1.0
        v = p.tune.retreat_speed
        vx, vy = v * ux / n, v * uy / n
        self.retreat_travel += v * dt
        return vx, vy, wz, None

    def _act_edge(self):
        """§4.7。向きは変えずに中央へ戻る。相手が見えていれば正面に保つ。"""
        cx, cy = self.pose.to_center()
        v = self.p.tune.edge_speed
        wz = self._servo(self.tracker.bearing) if self.tracker.visible else 0.0
        return v * cx, v * cy, wz, None

    # ================================================================ 出力
    def _fill_debug(self, dbg, d_cliff):
        tr, pose = self.tracker, self.pose
        dbg.setdefault('rho', tr.rng if tr.rng is not None else float('nan'))
        dbg.setdefault('beta', tr.bearing if tr.bearing is not None else float('nan'))
        dbg['z_top'] = dbg.get('z_top', float('nan'))
        dbg['H_o'] = self.fall.h_stand
        dbg['fallen'] = 1.0 if self.fall.fallen else 0.0
        dbg['d_edge'] = pose.d_edge
        dbg['d_margin'] = pose.d_margin
        dbg['d_cliff0'] = d_cliff if d_cliff is not None else float('nan')
        dbg['x_r'], dbg['y_r'], dbg['psi'] = pose.x, pose.y, pose.yaw
        dbg['s'] = pose.s
        dbg['state_id'] = float(STATES.index(self.state))
        dbg['t_in_state'] = self.t_in_state
        dbg['t_since_obs'] = min(tr.t_since_obs, 99.0)
        dbg['t_since_move'] = self.keepalive.t_since_move
        dbg['vx'], dbg['vy'], dbg['wz'] = self.last_cmd

    def _command(self, vx, vy, wz, motion, dbg):
        return Command(vx=vx, vy=vy, wz=wz, motion=motion,
                       state=self.state, reason=self.reason, debug=dbg)
