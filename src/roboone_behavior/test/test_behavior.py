# -*- coding: utf-8 -*-
"""行動層の単体テスト。

docs/behavior_planning.pdf §6.2 の「観測列を与えれば決定的に動く」を使って、
文書が決めた状態遷移と、規則から来る制約を固定する。狙いは実機の正しさの
主張ではなく、「文書が挙げた条件から外れないこと」の据え付けである。

    §2.2  自分が動いている間も、静止した相手は静止して見える
    §2.3  転倒判定のヒステリシス。間合いの中では横幅の証拠を併せて要る（変更点）
    §2.4  縁までの距離が式 (5) と一致する
    §3.1  優先順位。WAIT / EDGE がどの状態にも割り込む
    §4.3  ρ_s の手前で止まる
    §4.5  技は 1 回だけ出て、再生中は歩行指令を出さない
    §4.6  相手が転倒したら ρ_r まで離れる（規則 10.2(b)(i)）
    §4.8  先読みで縁へ向かう成分だけ削る。中央へ戻る成分は削らない（変更点）
    §4.9  10 秒以上前後左右に移動しない状態を作らない（規則 10.3(d)）
    §4.5  しゃがみを含む技のあとは 3 歩以上歩く（規則 10.2(l)）

ROS は要らない。roboone_perception の test_detect.py と同じで、計算部分だけを
直接叩く。
"""

import math

import pytest
from roboone_behavior.behavior import (APPROACH, BehaviorCore, BehaviorParams,
                                       distance_to_edge, EDGE, ENGAGE,
                                       FallenDetector, Observation,
                                       OpponentTracker, ray_to_edge, RETREAT,
                                       RingPose, SEARCH, WAIT)
import sim as S

DT = 0.05


def core(**robot):
    """自陣コーナーではなく、指定した位置から始まる core を作る。"""
    p = BehaviorParams()
    for k, v in robot.items():
        setattr(p.robot, k, v)
    return BehaviorCore(p)


# ============================================================ §2.4 リングの幾何
def test_edge_distance_matches_closed_form():
    """式 (5) と一致すること。a=1.8, c=0.9 の八角形。"""
    m = BehaviorParams().match
    a, c = m.ring_half_width, m.ring_corner_cut
    for x, y in ((0, 0), (1.0, 0.3), (-1.5, 0.9), (1.2, 1.2), (0.0, -1.7)):
        want = min(a - abs(x), a - abs(y),
                   (2 * a - c - abs(x) - abs(y)) / math.sqrt(2.0))
        assert distance_to_edge(m, x, y) == pytest.approx(want, abs=1e-9)


def test_edge_distance_is_zero_on_the_boundary():
    m = BehaviorParams().match
    assert distance_to_edge(m, 1.8, 0.0) == pytest.approx(0.0, abs=1e-9)
    # 角の辺は |x| + |y| = 2a - c = 2.7
    assert distance_to_edge(m, 1.35, 1.35) == pytest.approx(0.0, abs=1e-9)
    assert distance_to_edge(m, 2.0, 0.0) < 0.0


def test_ray_to_edge_agrees_with_walking_there():
    """視線方向の距離だけ進むと、ちょうど縁に着くこと。"""
    m = BehaviorParams().match
    for theta in (0.0, 0.7, 2.4, -1.9):
        d = ray_to_edge(m, 0.3, -0.4, theta)
        x = 0.3 + d * math.cos(theta)
        y = -0.4 + d * math.sin(theta)
        assert distance_to_edge(m, x, y) == pytest.approx(0.0, abs=1e-6)


# ============================================================ §4.8 先読みの縮小
def test_lookahead_shrinks_command_towards_the_edge():
    p = BehaviorParams()
    pose = RingPose(p.match, p.robot, p.tune)
    # 東の縁 (d_edge=0.3) を向いている。1 秒先の 1.65 は余裕 0.25 の外側になる
    pose.x, pose.y, pose.yaw = 1.5, 0.0, 0.0
    k = pose.clamp_lookahead(0.15, 0.0, 1.0)
    assert 0.0 <= k < 1.0
    # 縮めた後の 1 秒先が余裕の内側にあること
    assert distance_to_edge(p.match, pose.x + k * 0.15, pose.y) >= pose.d_margin - 1e-6


def test_lookahead_does_not_block_escaping_from_outside_the_margin():
    """文書からの変更点。余裕を割った位置でも中央へ戻る指令は削らない。

    字義どおりの「余裕の内側でなければ縮める」だと、既に割っている位置では
    EDGE の指令までゼロになり、縁から動けなくなる（ring.py 冒頭の注記）。
    """
    p = BehaviorParams()
    pose = RingPose(p.match, p.robot, p.tune)
    pose.x, pose.y, pose.yaw = 1.75, 0.0, math.pi   # 縁の際で、中央を向いている
    assert pose.d_edge < pose.d_margin
    assert pose.clamp_lookahead(0.1, 0.0, 1.0) == pytest.approx(1.0)
    # 逆に、さらに縁へ向かう指令は通さない
    pose.yaw = 0.0
    assert pose.clamp_lookahead(0.1, 0.0, 1.0) == pytest.approx(0.0, abs=1e-3)


# ============================================================ §2.2 相手の追跡
def test_tracker_holds_a_static_opponent_while_the_robot_turns():
    """式 (2)。自分が回っている間、観測が来なくても相手は同じ場所に居続ける。"""
    p = BehaviorParams()
    tr = OpponentTracker(p.tune, p.robot)
    tr.step((1.0, 0.0), (0.0, 0.0, 0.0), DT)        # 正面 1 m で初観測
    wz = 0.6
    for _ in range(10):                              # 0.5 s、観測なしで旋回
        tr.step(None, (0.0, 0.0, wz), DT)
    # 機体が +0.3 rad 回ったぶん、相手は機体座標で -0.3 rad へ動いて見える
    assert tr.rng == pytest.approx(1.0, abs=1e-6)
    assert tr.bearing == pytest.approx(-wz * 10 * DT, abs=1e-6)


def test_tracker_rejects_a_single_outlier_but_follows_a_persistent_jump():
    p = BehaviorParams()
    tr = OpponentTracker(p.tune, p.robot)
    tr.step((1.0, 0.0), (0, 0, 0), DT)
    tr.step((1.0, 1.0), (0, 0, 0), DT)               # ゲート外。1 回目は捨てる
    assert tr.pos[1] == pytest.approx(0.0, abs=1e-6)
    tr.step((1.0, 1.0), (0, 0, 0), DT)               # 2 回続いた。乗り換える
    assert tr.pos[1] == pytest.approx(1.0, abs=1e-6)


def test_tracker_reports_a_loss_after_the_lost_time():
    p = BehaviorParams()
    tr = OpponentTracker(p.tune, p.robot)
    tr.step((1.0, 0.0), (0, 0, 0), DT)
    assert tr.visible
    for _ in range(int(p.tune.lost_time / DT) + 1):
        tr.step(None, (0, 0, 0), DT)
    assert not tr.visible and tr.active           # 見失っても位置は保つ


# ============================================================ §2.3 転倒判定
def _feed(det, z, w, rng, seconds):
    for _ in range(int(seconds / DT)):
        det.step(z, w, rng, DT)
    return det.fallen


def test_fall_detection_far_away_uses_height_hysteresis():
    p = BehaviorParams()
    det = FallenDetector(p.tune, p.robot)
    _feed(det, 0.35, 0.20, 1.5, p.tune.height_cal_time + 0.2)   # H_o を測る
    assert det.h_stand == pytest.approx(0.35, abs=1e-6)
    assert not det.fallen
    assert _feed(det, 0.12, 0.40, 1.5, p.tune.fallen_time + 0.1)
    # 復帰は κ_u = 0.75 を超えてから T_up 続いたとき
    assert not _feed(det, 0.30, 0.20, 1.5, p.tune.stand_time + 0.1)


def test_fall_detection_needs_the_width_evidence_inside_the_strike_range():
    """文書からの変更点（tracking.py の _step_close の注記）。

    間合いの中では上端が視野で切れて低く出る。高さだけで転ばせない。
    """
    p = BehaviorParams()
    close = p.robot.strike_range + p.tune.fallen_freeze_margin - 0.05
    det = FallenDetector(p.tune, p.robot)
    _feed(det, 0.35, 0.20, 1.5, p.tune.height_cal_time + 0.2)
    # 上端だけ低い（＝視野で切れただけ）では転倒にしない
    assert not _feed(det, 0.12, 0.20, close, p.tune.fallen_time + 0.5)
    # 横にも広がったら転倒とみなす
    assert _feed(det, 0.12, 0.40, close, p.tune.fallen_time + 0.1)


def test_recovery_stays_frozen_inside_the_strike_range():
    """近いままでは復帰を認めない。離れてから判定する。"""
    p = BehaviorParams()
    close = p.robot.strike_range + 0.05
    det = FallenDetector(p.tune, p.robot)
    _feed(det, 0.35, 0.20, 1.5, p.tune.height_cal_time + 0.2)
    assert _feed(det, 0.12, 0.40, 1.5, p.tune.fallen_time + 0.1)
    assert _feed(det, 0.35, 0.20, close, 2.0)          # 近いので凍結。転倒のまま
    assert not _feed(det, 0.35, 0.20, 1.5, p.tune.stand_time + 0.1)


# ============================================================ §3.1 優先順位
def test_wait_until_autonomy_and_no_command_at_all():
    c = core()
    for _ in range(20):
        cmd = c.update(Observation(dt=DT, autonomy=False))
    assert cmd.state == WAIT
    assert (cmd.vx, cmd.vy, cmd.wz) == (0.0, 0.0, 0.0)


def test_estop_returns_to_wait_from_any_state():
    c = core(start_x=-1.2, start_y=0.0, start_yaw=0.0)
    s = S.Scene(c, opponent=(0.0, 0.0))
    s.run(2.0)
    assert c.state == APPROACH
    cmd = s.step(DT, estop=True)
    assert cmd.state == WAIT and (cmd.vx, cmd.vy, cmd.wz) == (0.0, 0.0, 0.0)


def test_edge_preempts_and_walks_back_towards_the_centre():
    """縁の余裕を割ったら、他の状態に優先して中央へ戻る。"""
    c = core(start_x=-1.70, start_y=0.0, start_yaw=math.pi)
    s = S.Scene(c, opponent=(1.0, 0.0))
    cmd = s.step()
    assert cmd.state == EDGE
    d0 = c.pose.d_edge
    s.run(4.0)
    assert c.pose.d_edge > d0
    # d_1 まで戻ったら解除される
    s.run(4.0)
    assert c.state != EDGE and c.pose.d_edge >= c.p.tune.edge_release


# ============================================================ §4.3 接近
def test_approach_stops_at_the_strike_range():
    c = core(start_x=-1.5, start_y=0.0, start_yaw=0.0)
    s = S.Scene(c, opponent=(0.0, 0.0))
    s.run(20.0)
    rng, _, _ = s.polar()
    p = c.p
    assert rng <= p.robot.strike_range + p.tune.range_eps + 0.05
    assert s.entered(APPROACH) and s.entered(ENGAGE)


def test_search_turns_towards_the_side_the_opponent_was_last_seen():
    """§4.2。最後に見えた方位の側へ回る。"""
    c = core(start_x=0.0, start_y=0.0, start_yaw=0.0)
    s = S.Scene(c, opponent=(0.5, 1.2))          # 左前方 (β > 0)
    s.run(0.5)
    cmd = s.run(1.0, drop=True)                  # 見失わせる
    assert cmd.state == SEARCH
    assert cmd.wz > 0.0                          # 左へ回る


# ============================================================ §4.5 技
def test_engage_fires_one_technique_and_is_silent_while_it_plays():
    c = core(start_x=-0.6, start_y=0.0, start_yaw=0.0)
    s = S.Scene(c, opponent=(0.0, 0.0))
    s.run(6.0)                                   # 0.6 m から間合いまで詰めるのに要る
    assert len(s.fired) >= 1
    first = s.fired[0]
    # 再生中 (1.0 s) は 1 回しか出ていない
    fired_during = [f for f in s.fired if first[0] <= f[0] < first[0] + 1.0]
    assert len(fired_during) == 1


def test_no_walk_command_while_a_technique_plays():
    c = core(start_x=-0.6, start_y=0.0, start_yaw=0.0)
    seen = []

    def watch(scene, cmd):
        if scene.busy_left > 0:
            seen.append((cmd.vx, cmd.vy, cmd.wz))

    s = S.Scene(c, opponent=(0.0, 0.0))
    s.run(6.0, on_step=watch)
    assert seen and all(v == (0.0, 0.0, 0.0) for v in seen)


def test_crouch_technique_forces_three_steps_before_the_next_one():
    """規則 10.2(l)。しゃがみを含む技のあとは 3 歩以上歩いてから。"""
    p = BehaviorParams()
    p.robot.start_x, p.robot.start_y, p.robot.start_yaw = -0.6, 0.0, 0.0
    p.robot.techniques = ['squat_punch']
    p.robot.crouch_techniques = ['squat_punch']
    c = BehaviorCore(p)
    s = S.Scene(c, opponent=(0.0, 0.0))
    s.run(12.0)
    assert len(s.fired) >= 2
    gap = s.fired[1][0] - s.fired[0][0]
    # 技の再生 1.0 s に加えて、3 歩ぶん (3 × 0.4 s) 以上あくこと
    assert gap >= 1.0 + p.match.crouch_steps * p.robot.step_period


# ============================================================ §4.6 離れる
def test_retreat_backs_off_when_the_opponent_goes_down():
    c = core(start_x=-1.2, start_y=0.0, start_yaw=0.0)
    s = S.Scene(c, opponent=(0.0, 0.0))
    s.run(9.0)                                   # 間合いまで詰める
    assert c.state in (APPROACH, ENGAGE)
    s.top, s.width = 0.12, 0.45                  # 相手が倒れる
    s.run(6.0)
    assert s.entered(RETREAT)
    rng, _, _ = s.polar()
    assert rng >= c.p.match.retreat_range - 0.05


def test_retreat_releases_when_the_opponent_gets_up():
    c = core(start_x=-1.2, start_y=0.0, start_yaw=0.0)
    s = S.Scene(c, opponent=(0.0, 0.0))
    s.run(9.0)
    s.top, s.width = 0.12, 0.45
    s.run(4.0)
    assert c.state == RETREAT
    s.top, s.width = 0.35, 0.20                  # 起き上がった
    s.run(3.0)
    assert c.state != RETREAT


# ============================================================ §4.9 止まらない
def test_never_stands_still_longer_than_the_keepalive_period():
    """規則 10.3(d)。前後左右に動かない時間が T_ka を大きく超えないこと。

    相手が転倒したまま復帰しない、行動としては一番長く止まる場面で見る。
    """
    c = core(start_x=-1.2, start_y=0.0, start_yaw=0.0)
    s = S.Scene(c, opponent=(0.0, 0.0), top=0.12, width=0.45)
    worst = [0.0]

    def watch(scene, cmd):
        worst[0] = max(worst[0], c.keepalive.t_since_move)

    s.run(40.0, on_step=watch)
    assert c.state == RETREAT
    # 規則の 10 s に対して余裕があること。T_ka=6 s + 横移動 1 回ぶん
    assert worst[0] < 8.0


def test_wait_does_not_add_the_keepalive_motion():
    """WAIT では足踏みも小移動もしない（規則 10.2(k)）。"""
    c = core()
    for _ in range(int(30.0 / DT)):
        cmd = c.update(Observation(dt=DT, autonomy=False))
        assert (cmd.vx, cmd.vy, cmd.wz) == (0.0, 0.0, 0.0)


# ============================================================ 縮退した知覚
def test_search_slows_down_when_the_attitude_is_degraded():
    """Opponent.status が縮退を報せている間は旋回を落とす。"""
    p = BehaviorParams()
    fast = BehaviorCore(p)
    obs = Observation(dt=DT, autonomy=True, opponent_fresh=True, opponent_status=1)
    for _ in range(4):
        a = fast.update(obs)
    slow = BehaviorCore(BehaviorParams())
    obs2 = Observation(dt=DT, autonomy=True, opponent_fresh=True, opponent_status=2)
    for _ in range(4):
        b = slow.update(obs2)
    assert abs(b.wz) < abs(a.wz)


# ============================================================ motion との接点
def test_parses_the_motion_state_strings_the_real_node_emits():
    """実機の motion ノードが出す書式（motion_node.cpp の publishState）。

    技の再生中だけコロン区切りで技名が付く。ここを読み違えると、技を再生して
    いる最中に歩行指令を重ねることになる。
    """
    from roboone_behavior.behavior_node import _parse_motion_state
    busy = {'MOTION', 'PLAYING', 'BUSY'}
    assert _parse_motion_state('RELAX', busy) == ('RELAX', False)
    assert _parse_motion_state('ARMING', busy) == ('ARMING', False)
    assert _parse_motion_state('HOLD', busy) == ('HOLD', False)
    assert _parse_motion_state('WALK', busy) == ('WALK', False)
    assert _parse_motion_state('MOTION:punch_r', busy) == ('MOTION', True)
    # 歩行ノート(2) §7.4 の「状態・支持脚・位相」が後から足されても落ちない
    assert _parse_motion_state('STEP support=L phase=0.30', busy) == ('STEP', False)
    assert _parse_motion_state('STEP motion=getup_front', busy) == ('STEP', True)
    assert _parse_motion_state('', busy) == ('IDLE', False)


def test_holds_still_while_motion_cannot_walk():
    """RELAX / ARMING の間は歩行指令を出さず、位置の推定も進めない。

    出しても機体は動かないのに、リング座標系は指令の積分で進むので、
    「歩いたつもり」で縁の判定がずれる。
    """
    c = core(start_x=-1.2, start_y=0.0, start_yaw=0.0)
    s = S.Scene(c, opponent=(0.0, 0.0))
    s.motion_state = 'RELAX'
    s.run(3.0)
    assert c.state == 'SELF_DOWN'
    assert (c.pose.x, c.pose.y) == pytest.approx((-1.2, 0.0))
    # トルクが入って HOLD になったら動き出す
    s.motion_state = 'HOLD'
    s.run(2.0)
    assert c.state == APPROACH and c.pose.x > -1.2
