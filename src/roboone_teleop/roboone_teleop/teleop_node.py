# -*- coding: utf-8 -*-
"""teleop ノード — PS5 (DualSense) コントローラで機体を無線操縦する。

ros-architecture §2/§4 の約束事:

    受け取る: /joy (sensor_msgs/Joy)          … joy パッケージのノードが出す
    出す:     /estop (std_msgs/Bool)          … 脱力 / トルクオン。イベント時
              /cmd_walk (geometry_msgs/Twist) … 歩行指令。20Hz
              /cmd_motion (std_msgs/String)   … 技名。イベント時
              /ui/led, /ui/buzzer             … 無線テストのときだけ (§無線テスト)

指令の対応 (割り当ての既定値は config/ps5_dualsense.yaml):

    並行移動      左スティック  → linear.x / linear.y。**angular.z は載せない**ので
                                 斜め前・斜め後ろへは機体の向きを変えずに歩く
    旋回          右スティック左右 → angular.z
    パンチ 右/左  /cmd_motion "punch_r" / "punch_l"
    起き上がり    /cmd_motion "getup_front" / "getup_back"
    脱力          /estop true   … ラッチする。ウォッチドッグの発報先もここ
    トルクオン    /estop false  … 長押し。誤操作で急にトルクが入らないようにする
    無線テスト    押している間だけブザーを鳴らし LED の色を変える

**脱力とトルクオンは /cmd_motion ではなく /estop に載せる。** 技名で送ると
「非常停止がラッチされているのに torque_on が届く」という矛盾した状態を motion 側で
解く羽目になる。トルクの ON/OFF は経路を 1 本に絞って、非常停止・ウォッチドッグ・
手動の脱力を全部そこへ集める。motion 側の約束は
「/estop true を受けたら即トルクOFF、false を受けたらトルクON」の 1 行で済む。

安全側の設計 (無線なので、ここが本体):

  * **デッドマン**: R1 を押している間しか /cmd_walk に値が乗らない。離せばゼロ。
    「押している間だけ動く」であって「押すと動き出す」ではない。
  * **無通信ウォッチドッグ**: /joy が joy_timeout 秒途切れたら非常停止をラッチする。
    Bluetooth が切れる・電池が切れる・コントローラを踏む、はどれも実際に起きる。
    joy 側の autorepeat_rate を 0 より大きくしておくこと (launch で設定済み)。
    スティックを動かさない限り /joy が来ない設定だと、静止＝断線と見分けが付かない。
  * **起動時の再武装**: 起動直後・非常停止解除直後は、デッドマンを一度離すまで
    歩行指令を受け付けない。ボタンを押したままの状態で復帰していきなり歩き出す事故を防ぐ。
  * **加速度制限**: スティックの段差をそのまま速度指令にせず、1 周期あたりの
    変化量を制限する。二足で速度指令が階段状に飛ぶと、それだけで転ぶ。
  * **常時 20Hz 送信**: 止まっているときもゼロを送り続ける。無送信で「最後の指令が
    残る」より、ゼロが来続けるほうが motion 側の実装が単純で安全になる。

無線テストだけは上の制約の外にある。**デッドマン不要・脱力中でも動く**。
機体を安全な脱力状態に置いたまま、離れた場所で電波が届いているかを確かめるための
機能なので、動作条件を付けると用を成さない。

ボタン割り当ては config/ps5_dualsense.yaml。index を直接書いてあるので、
実機で `ros2 run roboone_teleop joy_probe` を回して確認してから使うこと。
"""

import math
import signal
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from roboone_interfaces.msg import LedColor
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

from .bindings import apply_deadzone, Binding, parse_motion_bindings

#: /estop は latched。teleop より後に motion を起動しても、直前の脱力状態が届く。
#: ここを Volatile にすると「脱力させた状態で motion を再起動したら動き出した」
#: が起こりうる。ui ノードの購読側も同じ設定なので、/ui/* もこの profile で出す
#: (VOLATILE な publisher は TRANSIENT_LOCAL な subscriber とマッチしない)。
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class TeleopNode(Node):

    def __init__(self, **kwargs):
        # kwargs は rclpy.node.Node にそのまま渡す。テストから parameter_overrides を
        # 差し込むためだけの口で、本番の main() では何も渡さない。
        super().__init__('teleop', **kwargs)

        # --- パラメータ -----------------------------------------------------
        p = self.declare_parameter
        self._rate_hz = p('rate_hz', 20.0).value              # /cmd_walk の送信周期
        self._joy_timeout = p('joy_timeout', 0.5).value       # これだけ無通信で非常停止
        self._deadzone = p('deadzone', 0.12).value

        self._ax = {
            'x':   p('axes.walk_x', 1).value,                 # 左スティック 上下 → 前後
            'y':   p('axes.walk_y', 0).value,                 # 左スティック 左右 → 横歩き
            'yaw': p('axes.walk_yaw', 2).value,               # 右スティック 左右 → 旋回
        }
        self._inv = {
            'x':   p('invert.walk_x', True).value,
            'y':   p('invert.walk_y', True).value,
            'yaw': p('invert.walk_yaw', True).value,
        }
        self._scale = {
            'x':   p('scale.x', 0.06).value,                  # m/s
            'y':   p('scale.y', 0.03).value,                  # m/s
            'yaw': p('scale.yaw', 0.40).value,                # rad/s
        }
        self._accel = {
            'x':   p('accel.x', 0.15).value,                  # m/s^2
            'y':   p('accel.y', 0.10).value,
            'yaw': p('accel.yaw', 1.50).value,                # rad/s^2
        }
        self._b_deadman = Binding(p('buttons.deadman', 'b10').value)      # R1
        self._b_estop_set = Binding(p('buttons.relax', 'b9').value)       # L1 脱力
        self._b_estop_clear = Binding(p('buttons.torque_on', 'b6').value)  # Options
        self._clear_hold = p('torque_on_hold', 1.0).value

        self._motion_bindings = parse_motion_bindings(
            p('motion_bindings',
              ['b1:punch_r', 'b2:punch_l', 'b3:getup_front', 'b0:getup_back']).value)
        self._motion_needs_deadman = p('motion_requires_deadman', True).value
        self._motion_cooldown = p('motion_cooldown', 0.5).value

        # 無線テスト。押している間だけブザーと LED で「届いている」ことを示す。
        self._b_link = Binding(p('buttons.link_test', 'b4').value)        # Create
        self._link_color = _rgb(p('link_test.color', [0, 200, 255]).value)
        self._link_off_color = _rgb(p('link_test.release_color', [0, 0, 0]).value)
        self._link_buzzer = p('link_test.buzzer', 'beep').value
        self._link_hz = p('link_test.buzzer_hz', 15.0).value
        # 無線テストだけは joy_timeout (0.5s) では遅い。電波の切れ目を耳で探すのが
        # 目的なので、数フレーム落ちた時点で鳴り止ませる。20Hz の autorepeat なら
        # 0.15s = 3 フレームぶん。
        self._link_stale = p('link_test.stale', 0.15).value

        # --- 状態 -----------------------------------------------------------
        self._joy = None            # 最新の Joy。まだ一度も来ていなければ None
        self._joy_stamp = None      # 最後に /joy を受けた時刻 (秒, ROS time)
        self._estop = False
        self._estop_reason = ''
        self._armed = False         # デッドマンを一度離すまで False
        self._clear_since = None    # 解除ボタンを押し始めた時刻
        self._prev_motion = {}      # 技ボタンの前フレームの押下状態 (立ち上がり検出用)
        self._motion_until = 0.0    # 連射防止
        self._cmd = [0.0, 0.0, 0.0]  # 実際に出している (x, y, yaw)。加速度制限後の値
        self._estop_beat = 0
        self._warned_short = False
        self._link = False          # 無線テストのボタンを押しているか
        self._link_next = 0.0       # 次にブザーを鳴らし直す時刻

        # --- 通信 -----------------------------------------------------------
        self._pub_walk = self.create_publisher(Twist, '/cmd_walk', 10)
        self._pub_estop = self.create_publisher(Bool, '/estop', LATCHED)
        self._pub_motion = self.create_publisher(String, '/cmd_motion', 10)
        # 無線テスト用。ui ノードの購読側が latched なので、こちらも latched で出す。
        self._pub_led = self.create_publisher(LedColor, '/ui/led', LATCHED)
        self._pub_buzzer = self.create_publisher(String, '/ui/buzzer', LATCHED)
        self.create_subscription(Joy, '/joy', self._on_joy, 10)

        # 起動直後に「非常停止していない」を 1 回流しておく。latched なので、
        # 後から起動した motion はこれを受けて初期状態を確定できる。
        self._publish_estop()

        self._timer = self.create_timer(1.0 / self._rate_hz, self._tick)
        self.get_logger().info(
            f'teleop 起動。デッドマン={self._b_deadman.spec} '
            f'脱力={self._b_estop_set.spec} '
            f'トルクオン={self._b_estop_clear.spec}({self._clear_hold:.1f}s 長押し) '
            f'無線テスト={self._b_link.spec} '
            f'joy_timeout={self._joy_timeout:.2f}s')

    # ------------------------------------------------------------------ 時刻
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -------------------------------------------------------------- Joy 受信
    def _on_joy(self, msg: Joy):
        self._joy = msg
        was_stale = self._joy_stamp is None
        self._joy_stamp = self._now()
        if was_stale:
            self.get_logger().info(
                f'/joy 受信開始 (軸 {len(msg.axes)} 本 / ボタン {len(msg.buttons)} 個)')

    # ------------------------------------------------------------ 周期処理
    def _tick(self):
        now = self._now()

        # 1) ウォッチドッグ。一度でも /joy が来たあとで途切れたら非常停止。
        #    一度も来ていないうちは「まだ joy_node が上がっていない」なので待つ。
        if self._joy_stamp is not None and now - self._joy_stamp > self._joy_timeout:
            self._set_estop(f'/joy が {now - self._joy_stamp:.2f}s 途切れた')
            self._joy = None

        target = [0.0, 0.0, 0.0]
        link = False
        if self._joy is not None:
            axes, buttons = self._joy.axes, self._joy.buttons
            self._check_length(axes, buttons)

            # 2) 脱力 / トルクオン。脱力は押した瞬間、トルクオンは長押し。
            if self._b_estop_set.pressed(axes, buttons):
                self._set_estop('脱力ボタン')
            self._handle_clear(axes, buttons, now)
            link = (self._b_link.pressed(axes, buttons)
                    and now - self._joy_stamp <= self._link_stale)

            # 3) 再武装。デッドマンを離した状態を一度見るまで指令を通さない。
            deadman = self._b_deadman.pressed(axes, buttons)
            if not deadman:
                if not self._armed and not self._estop:
                    self._armed = True
                    self.get_logger().info('デッドマン再武装。R1 で歩行指令が出せる')
            elif self._armed and not self._estop:
                # 左スティックは linear だけ、右スティックは angular だけ。混ぜない。
                # 斜めに歩くときに機体が勝手に旋回すると、狙った方向へ進まなくなる。
                for i, k in enumerate(('x', 'y', 'yaw')):
                    target[i] = self._axis(axes, k) * self._scale[k]

            # 4) 技指令。押した瞬間だけ 1 回送る。
            self._handle_motion(axes, buttons, now, deadman)

        # 4.5) 無線テスト。デッドマンも要らず、脱力中でも動く (docstring 参照)。
        #      /joy が途切れたら link=False になり、鳴り止む。
        self._handle_link(link, now)

        # 5) 加速度制限を掛けて送信。非常停止中もゼロを送り続ける (無送信にしない)。
        dt = 1.0 / self._rate_hz
        for i, k in enumerate(('x', 'y', 'yaw')):
            self._cmd[i] = _slew(self._cmd[i], target[i], self._accel[k] * dt)

        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = self._cmd
        self._pub_walk.publish(msg)

        # 6) 非常停止中は 2 秒に 1 回だけ再送。latched QoS があるので本来不要だが、
        #    後から繋いだ購読者や再接続時の取りこぼしに対する保険。
        self._estop_beat += 1
        if self._estop and self._estop_beat >= max(1, int(self._rate_hz * 2)):
            self._estop_beat = 0
            self._publish_estop()

    # ------------------------------------------------------------ 補助メソッド
    def _axis(self, axes, key) -> float:
        i = self._ax[key]
        v = axes[i] if 0 <= i < len(axes) else 0.0
        v = apply_deadzone(v, self._deadzone)
        return -v if self._inv[key] else v

    def _check_length(self, axes, buttons):
        need_ax = max(self._ax.values())
        if need_ax >= len(axes) and not self._warned_short:
            self._warned_short = True
            self.get_logger().warn(
                f'Joy の軸が {len(axes)} 本しかないのに index {need_ax} を参照している。'
                'config の割り当てが機種に合っていない可能性が高い '
                '(joy_probe で確認すること)')

    def _set_estop(self, reason: str):
        if self._estop:
            return
        self._estop = True
        self._estop_reason = reason
        self._armed = False
        self._cmd = [0.0, 0.0, 0.0]   # 減速ではなく即ゼロ。脱力なので。
        self._clear_since = None
        self._estop_beat = 0
        self._publish_estop()
        self.get_logger().error(f'*** 脱力 (トルクOFF) *** 理由: {reason}')

    def _handle_clear(self, axes, buttons, now):
        if not self._estop:
            self._clear_since = None
            return
        if not self._b_estop_clear.pressed(axes, buttons):
            self._clear_since = None
            return
        if self._clear_since is None:
            self._clear_since = now
            return
        if now - self._clear_since >= self._clear_hold:
            self._estop = False
            self._estop_reason = ''
            self._clear_since = None
            self._armed = False       # 解除しても、デッドマンを離すまでは動かさない
            self._publish_estop()
            self.get_logger().warn('トルクオン。デッドマンを一度離すと歩行指令を受け付ける')

    def _handle_link(self, down, now):
        """無線テスト — 押している間ブザーを鳴らし、LED の色を変える。

        ブザーの再送が要るのは、ui ノードのブザープリセットが「1 回鳴って止まる」
        設計だから (roboone_ui のブリーフ)。押しっぱなし用の長いパターンを ui 側に
        足すこともできるが、短いプリセットを撃ち続けるほうが安全側に倒れる:
        teleop が落ちても電波が切れても、次の 1 発が来ないので 100ms 以内に鳴り止む。
        長いパターンだと、送った側が消えたあともその長さぶん鳴り続ける。
        """
        if down and not self._link:
            self._link = True
            self._link_next = 0.0
            self._pub_led.publish(self._link_color)
            self.get_logger().info('無線テスト 開始 (電波が届いている)')
        elif not down and self._link:
            self._link = False
            self._pub_led.publish(self._link_off_color)
            self.get_logger().info('無線テスト 終了')

        if self._link and now >= self._link_next:
            self._link_next = now + 1.0 / self._link_hz
            self._pub_buzzer.publish(String(data=self._link_buzzer))

    def _handle_motion(self, axes, buttons, now, deadman):
        for binding, name in self._motion_bindings:
            down = binding.pressed(axes, buttons)
            was = self._prev_motion.get(binding.spec, False)
            self._prev_motion[binding.spec] = down
            if not (down and not was):
                continue                      # 立ち上がりだけ拾う
            if self._estop:
                self.get_logger().warn(f'脱力中のため技 "{name}" は送らない')
                continue
            if self._motion_needs_deadman and not deadman:
                self.get_logger().warn(f'デッドマン未押下のため技 "{name}" は送らない')
                continue
            if now < self._motion_until:
                continue
            self._motion_until = now + self._motion_cooldown
            self._pub_motion.publish(String(data=name))
            self.get_logger().info(f'/cmd_motion → {name}')

    def _publish_estop(self):
        self._pub_estop.publish(Bool(data=self._estop))

    def destroy_node(self):
        """落ちるときはゼロと脱力を置いていく。

        rclpy の既定シグナルハンドラは context を先に畳むので、その状態でここへ来ても
        publish は誰にも届かない。main() で既定ハンドラを外してあるのはこのため
        (context が生きているうちにここを通す)。送出を DDS に渡す時間だけ待つ。

        ただし**これは保険であって当てにするものではない。** teleop が kill -9 された
        場合や電源ごと落ちた場合は当然届かない。motion 側は「/cmd_walk が途切れたら
        止まる」を自前で持つこと (teleop は静止中もゼロを 20Hz 送り続けている)。
        """
        try:
            if rclpy.ok():
                self._pub_walk.publish(Twist())
                self._estop = True
                self._publish_estop()
                if self._link:
                    self._pub_led.publish(self._link_off_color)   # 色を消し忘れない
                time.sleep(0.05)
        except Exception:
            pass
        return super().destroy_node()


def _rgb(triple) -> LedColor:
    """[r, g, b] を LedColor にする。範囲外は黙って 0-255 に丸める。"""
    vals = list(triple) + [0, 0, 0]
    return LedColor(**{k: max(0, min(255, int(v)))
                       for k, v in zip(('r', 'g', 'b'), vals)})


def _slew(current: float, target: float, max_step: float) -> float:
    d = target - current
    if abs(d) <= max_step:
        return target
    return current + math.copysign(max_step, d)


def main(args=None):
    # 既定のシグナルハンドラを使わない。rclpy の既定は SIGINT/SIGTERM で真っ先に
    # context を畳むので、destroy_node() の「ゼロと脱力を置いていく」が publish
    # できずに終わる (おまけに rclpy.spin() が RCLError を投げてトレースバックが出る)。
    # 自前のフラグで抜けて、context が生きているうちに終了処理を通す。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TeleopNode()
    stopping = []

    def _request_stop(_signum, _frame):
        stopping.append(True)

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _request_stop)

    try:
        # 20Hz タイマーがあるので spin_once はすぐ戻る。timeout はシグナルを
        # 取りこぼさないための上限。
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
