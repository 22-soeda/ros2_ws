# -*- coding: utf-8 -*-
"""behavior ノード — 相手の位置とリングの縁から、歩行指令と技を決める。

ros-architecture §2/§4 と docs/behavior_planning.pdf §6 の約束事:

    受け取る: /opponent      (roboone_interfaces/Opponent)  約 15Hz
              /ring_edge     (std_msgs/Float32MultiArray)   同上
              /motion/state  (std_msgs/String)              状態変化時
              /odom          (nav_msgs/Odometry)            歩の境界ごと
              /autonomy      (std_msgs/Bool, latched)       teleop から
              /estop         (std_msgs/Bool, latched)       teleop から
    出す:     /cmd_walk        (geometry_msgs/Twist)        20Hz
              /cmd_motion      (std_msgs/String)            イベント時
              /behavior/state  (std_msgs/String)            状態変化時
              /behavior/debug  (std_msgs/Float64MultiArray) 20Hz

判断の中身は roboone_behavior.behavior（ROS 非依存）にある。このノードが持つのは
「トピックとパラメータの面倒を見る」ことだけで、判断は 1 つも持たない。


behavior_planning.pdf から変えたところ（理由つき）
--------------------------------------------------

**1. 「はじめ」「待て」「止め」を /match/cmd (String) ではなく /autonomy (Bool) で受ける。**
teleop 側が先に /autonomy で実装されている（roboone_teleop の docstring）。規則
5.1.2 の無線始動・停止機構としては同じもので、hold と stop の区別が無いだけである。
行動層はどちらでも WAIT に落ちるので、区別する必要が無い。脱力 (/estop) も同じ扱い。

**2. /cmd_walk は /autonomy が true の間しか publish しない。** 文書 §5.1 の
「start のときだけ behavior が出す」をそのまま実装したもの。teleop 側は逆に
自律中だけ黙るので、2 つが同時に出す瞬間が無い。落ちる瞬間だけ 0 を 1 通出して
から黙る（最後に出した速度が motion に残らないように）。

**3. /odom が無くても動く。** 文書は /odom を歩行ノードの新設としているが、
motion ノードはまだ書かれていない。/odom が来ないうちは、自分が出した指令を
積分してリング座標系を進める（roboone_behavior.behavior.ring.RingPose）。滑りも
指令と実際のずれも式 (6) の余裕 d_margin が吸収する前提なので、精度の性質は
同じで、係数 κ_s が大きくなるだけである。/odom が来ればそちらへ自動で乗り換える。

**4. /motion/state の書式は決め打ちにしない。** 歩行ノートは「状態・支持脚・
位相を文字列で」としか決めていない。先頭の語を状態、`key=value` を属性として
読み、技の再生中かどうかは motion_busy_tokens / `motion=<技名>` のどちらでも
拾えるようにしてある。motion ノードが固まったらここを詰めること。
"""

from dataclasses import fields, MISSING
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from roboone_interfaces.msg import Opponent
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray, String
from std_msgs.msg import MultiArrayDimension

from .behavior import (BehaviorCore, BehaviorParams, debug_array, DEBUG_ORDER,
                       MatchParams, Observation, RobotParams, STATUS_RING_LOST,
                       TuneParams)

#: teleop が /estop と /autonomy を latched で出すので、購読側も合わせる。
#: VOLATILE な購読者は TRANSIENT_LOCAL な publisher とマッチはするが、
#: behavior を teleop より後に上げたときに「直前の状態」が届かない。
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

#: /behavior/debug は落ちてよい。記録と PlotJuggler 向け
BEST_EFFORT = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

_GROUPS = (('match.', MatchParams), ('robot.', RobotParams), ('tune.', TuneParams))


def _default(f):
    """フィールドの既定値を取り出す。default_factory も潰して値にする。"""
    if f.default is not MISSING:
        return f.default
    return f.default_factory()


def _descriptor(value):
    """空リストの既定値に型を付ける。

    rclpy は既定値から型を推す。空リストは中身が無いので推せず、宣言した
    そばから ParameterUninitializedException になる（robot.crouch_techniques が
    これ）。この ws で空になりうるリストは技名の並びだけなので、文字列配列だと
    明示しておく。
    """
    if isinstance(value, (list, tuple)) and len(value) == 0:
        return ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
    return ParameterDescriptor()


class BehaviorNode(Node):

    def __init__(self, **kwargs):
        super().__init__('behavior', **kwargs)

        # --- トピックと入出力の設定 ------------------------------------
        self.declare_parameter('opponent_topic', '/opponent')
        self.declare_parameter('ring_edge_topic', '/ring_edge')
        self.declare_parameter('motion_state_topic', '/motion/state')
        self.declare_parameter('odom_topic', '/odom')
        # /opponent が途切れたら「知覚が死んでいる」として扱う。検出器は
        # 見えていない間も publish_lost() を出し続けるので、本当に来ないのは
        # ノードが落ちたときだけ（roboone_perception の同名メソッド参照）
        self.declare_parameter('opponent_timeout', 0.5)
        # /odom がこれだけ来なければ、指令の積分に戻る
        self.declare_parameter('odom_timeout', 1.0)
        # /motion/state のうち「技の再生中」を意味する語
        self.declare_parameter('motion_busy_tokens', ['MOTION', 'PLAYING', 'BUSY'])
        # /motion/state のうち「歩ける」状態。ここに無い状態（RELAX = 脱力、
        # ARMING = トルク投入中）の間は歩行指令を出さない。歩行ノート(2) の
        # 名前 (IDLE/START/STEP/STOP) も、motion 側が将来そちらを出しても
        # 動くように入れてある
        self.declare_parameter(
            'motion_ready_states',
            ['HOLD', 'WALK', 'MOTION', 'IDLE', 'START', 'STEP', 'STOP'])
        self.declare_parameter('publish_debug', True)

        # --- 行動層の定数。§5 の 3 群をそのまま宣言する -------------------
        for prefix, cls in _GROUPS:
            for f in fields(cls):
                d = _default(f)
                self.declare_parameter(prefix + f.name, d, _descriptor(d))

        self.core = BehaviorCore(self._collect_params())
        self.add_on_set_parameters_callback(self._on_params)

        self.opp_timeout = float(self.get_parameter('opponent_timeout').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.busy_tokens = {str(s).upper() for s
                            in self.get_parameter('motion_busy_tokens').value}
        self.ready_states = {str(s).upper() for s
                             in self.get_parameter('motion_ready_states').value}
        self.want_debug = bool(self.get_parameter('publish_debug').value)

        # --- 最新値の置き場。コールバックは置くだけで計算しない -------------
        self.opp = None            # 未処理の /opponent。取り出したら None に戻す
        self.opp_wall = None       # 最後に /opponent が来た時刻
        self.cliff = None
        self.cliff_half_fov = 0.0
        self.odom = None
        self.odom_wall = None
        self.motion_state = 'RELAX'   # 起動直後は歩けない側に倒す
        self.motion_busy = False
        self.autonomy = False
        self.estop = False
        self.last_tick = None
        self.last_state = None
        self.was_autonomous = False

        # --- 出入り口 ---------------------------------------------------
        self.pub_walk = self.create_publisher(Twist, '/cmd_walk', 10)
        self.pub_motion = self.create_publisher(String, '/cmd_motion', 10)
        self.pub_state = self.create_publisher(String, '/behavior/state', 10)
        self.pub_debug = self.create_publisher(Float64MultiArray,
                                               '/behavior/debug', BEST_EFFORT)

        self.create_subscription(Opponent,
                                 self.get_parameter('opponent_topic').value,
                                 self._on_opponent, 5)
        self.create_subscription(Float32MultiArray,
                                 self.get_parameter('ring_edge_topic').value,
                                 self._on_edge, 5)
        # motion ノードは /motion/state を latched で出す（motion_node.cpp の
        # latchedQos）。こちらも合わせておけば、behavior を後から上げても
        # 「今 RELAX なのか HOLD なのか」が即座に届く
        self.create_subscription(String,
                                 self.get_parameter('motion_state_topic').value,
                                 self._on_motion_state, LATCHED)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self._on_odom, 10)
        self.create_subscription(Bool, '/autonomy', self._on_autonomy, LATCHED)
        self.create_subscription(Bool, '/estop', self._on_estop, LATCHED)

        rate = float(self.core.p.tune.rate_hz)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

        r = self.core.p.robot
        self.get_logger().info(
            'behavior 起動: %.0f Hz / 間合い ρ_s=%.2f m / 上限 v=%.2f m/s ω=%.2f rad/s / '
            '開始位置 (%.2f, %.2f, %.0f deg)。/autonomy が true になるまで '
            '/cmd_walk は出さない'
            % (rate, r.strike_range, r.v_max, r.w_max, r.start_x, r.start_y,
               math.degrees(r.start_yaw)))

    # ------------------------------------------------------------ パラメータ
    def _collect_params(self):
        flat = {}
        for prefix, cls in _GROUPS:
            for f in fields(cls):
                flat[prefix + f.name] = self._value(prefix + f.name, _default(f))
        return BehaviorParams.from_flat(flat)

    def _value(self, name, default):
        """宣言済みパラメータの値。「空リスト」を既定値に畳んで返す。

        YAML に `crouch_techniques: []` と書くと、rclpy には中身から型を推せない
        PARAMETER_NOT_SET として届き、宣言時に型を付けてあっても上書きされて
        未初期化になる（ros2 の仕様）。空リストを書けないのは不便なので、
        未初期化と None はここで既定値に落とす。
        """
        try:
            v = self.get_parameter(name).value
        except ParameterUninitializedException:
            return default
        return default if v is None else v

    def _on_params(self, params):
        """実行時に変えてよいのは tune.* と robot.techniques だけ。

        match.* は規則とリングの寸法で、試合中に変わるものではない。
        robot.* の残りは機体の上限と開始位置で、走らせたまま変えると
        リング座標系の推定と食い違う（開始位置は「はじめ」の初期値そのもの）。
        """
        for p in params:
            if p.name.startswith('match.'):
                return SetParametersResult(
                    successful=False, reason='match.* は規則から決まる。起動時に固定する')
            if p.name.startswith('robot.') and not p.name.endswith(
                    ('techniques', 'crouch_techniques')):
                return SetParametersResult(
                    successful=False,
                    reason='robot.* は起動時に固定する。launch のパラメータで指定すること')
        # 値の反映は次周期。定数だけを差し替え、追跡と位置の推定は保つ
        self._pending = True
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ 購読
    def _on_opponent(self, msg):
        self.opp = msg
        self.opp_wall = self.get_clock().now().nanoseconds * 1e-9

    def _on_edge(self, msg):
        self.cliff = list(msg.data)
        # ビンの半角は dim のラベルに書いてある（検出器 _edge_msg の約束）。
        # 読めなければ検出器の既定 45 deg を使う
        self.cliff_half_fov = _half_fov_from_layout(msg) or math.radians(45.0)

    def _on_motion_state(self, msg):
        self.motion_state, self.motion_busy = _parse_motion_state(
            msg.data, self.busy_tokens)

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.odom_wall = self.get_clock().now().nanoseconds * 1e-9

    def _on_autonomy(self, msg):
        if bool(msg.data) != self.autonomy:
            self.get_logger().warn('/autonomy = %s' % bool(msg.data))
        self.autonomy = bool(msg.data)

    def _on_estop(self, msg):
        self.estop = bool(msg.data)

    # ------------------------------------------------------------ 1 周期
    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 1.0 / self.core.p.tune.rate_hz if self.last_tick is None \
            else max(now - self.last_tick, 1e-3)
        self.last_tick = now

        if getattr(self, '_pending', False):
            self._pending = False
            self.core.p = self._collect_params()

        obs = self._observe(now, dt)
        cmd = self.core.update(obs)

        active = self.autonomy and not self.estop
        if active:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = cmd.vx, cmd.vy, cmd.wz
            self.pub_walk.publish(t)
        elif self.was_autonomous:
            # 指令権を返す瞬間に 0 を 1 通。最後に出した速度を残さない
            self.pub_walk.publish(Twist())
        self.was_autonomous = active

        if cmd.motion:
            self.pub_motion.publish(String(data=cmd.motion))
            self.get_logger().info('技 -> %s' % cmd.motion)

        key = (cmd.state, cmd.reason)
        if key != self.last_state:
            self.last_state = key
            self.pub_state.publish(String(data='%s %s' % (cmd.state, cmd.reason)))
            self.get_logger().info('%s: %s' % (cmd.state, cmd.reason))

        if self.want_debug:
            self.pub_debug.publish(_debug_msg(cmd.debug))

    def _observe(self, now, dt):
        """最新値を Observation 1 個にまとめる。ここに判断は入れない。"""
        obs = Observation(dt=dt, autonomy=self.autonomy, estop=self.estop,
                          motion_state=self.motion_state,
                          motion_busy=self.motion_busy,
                          motion_ready=self.motion_state in self.ready_states)

        msg, self.opp = self.opp, None
        if msg is not None:
            obs.opponent_fresh = True
            obs.opponent_status = int(msg.status)
            obs.opponent_extrapolated = bool(msg.extrapolated)
            if msg.valid:
                obs.opponent_xy = (msg.position.x, msg.position.y)
                obs.opponent_top = float(msg.top_height)
                obs.opponent_width = float(msg.width)
        elif self.opp_wall is None or now - self.opp_wall > self.opp_timeout:
            # 検出器そのものが黙った。相手がいないのではなく知覚が死んでいる
            obs.opponent_fresh = True
            obs.opponent_status = STATUS_RING_LOST

        obs.cliff = self.cliff
        obs.cliff_half_fov = self.cliff_half_fov
        if self.odom_wall is not None and now - self.odom_wall <= self.odom_timeout:
            obs.odom = self.odom
        return obs


# ---------------------------------------------------------------- 小さな部品
def _parse_motion_state(text, busy_tokens):
    """/motion/state の文字列から (状態, 技の再生中か) を取り出す。

    実機の motion ノードが出すのは

        RELAX / ARMING / HOLD / WALK / MOTION:<技名>

    で、技の再生中だけ **コロン区切り**で技名が付く（motion_node.cpp の
    publishState）。歩行ノート(2) §7.4 は「状態・支持脚・位相を文字列で」と
    しか決めていないので、あとから支持脚や位相が空白や `key=value` で足されても
    落ちないように、先頭の語だけを状態として読む。

    技の再生中は「先頭の語が busy_tokens にある」か「コロンの後ろに技名が付いて
    いる」か「`motion=<技名>` がある」のどれでも拾う。
    """
    tokens = str(text).replace(',', ' ').split()
    if not tokens:
        return 'IDLE', False
    head, _, detail = tokens[0].partition(':')
    state = head.split('=')[-1].upper()
    busy = state in busy_tokens or bool(detail.strip())
    for tok in tokens[1:]:
        if tok.upper() in busy_tokens:
            busy = True
        if '=' in tok:
            k, v = tok.split('=', 1)
            if k.lower() in ('motion', 'playing') and v.lower() not in ('', 'none', '-'):
                busy = True
    return state, busy


def _half_fov_from_layout(msg):
    """dim.label の 'bearing_deg[-45.0,+45.0]' から半角 [rad] を読む。"""
    for dim in msg.layout.dim:
        if '[' not in dim.label or ',' not in dim.label:
            continue
        try:
            lo, hi = dim.label.split('[', 1)[1].rstrip(']').split(',')
            return math.radians(0.5 * (abs(float(lo)) + abs(float(hi))))
        except ValueError:
            continue
    return None


def _debug_msg(debug):
    msg = Float64MultiArray()
    dim = MultiArrayDimension()
    dim.label = ','.join(DEBUG_ORDER)
    dim.size = len(DEBUG_ORDER)
    dim.stride = len(DEBUG_ORDER)
    msg.layout.dim = [dim]
    msg.data = debug_array(debug)
    return msg


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 落ちる前に必ず 0 を 1 通。最後の速度を motion に残さない
        try:
            node.pub_walk.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
