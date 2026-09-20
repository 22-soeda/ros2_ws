# -*- coding: utf-8 -*-
"""teleop ノード — PS5 (DualSense) コントローラで機体を無線操縦する。

ros-architecture §2/§4 の約束事:

    受け取る: /joy (sensor_msgs/Joy)          … joy パッケージのノードが出す
    出す:     /estop (std_msgs/Bool)          … 脱力 / トルクオン。イベント時
              /cmd_walk (geometry_msgs/Twist) … 歩行指令。20Hz
              /cmd_motion (std_msgs/String)   … 技名。イベント時
              /autonomy (std_msgs/Bool)       … 自律動作の可否。イベント時

**本番での役割は非常停止と、そこからの復帰と、自律への指令権の受け渡し。**
手動操作 (デッドマン + 左スティック + 技) は、開発中に behavior を止めて歩行だけを
試すためのもの。この 4 つに入らない機能はここに置かない (診断は joy_probe と
teleop_params、状態表示は別ノードの仕事)。

指令の対応 (割り当ての既定値は config/ps5_dualsense.yaml):

    並行移動        左スティック  → linear.x / linear.y。**angular.z は載せない**ので
                                   斜め前・斜め後ろへは機体の向きを変えずに歩く。
                                   旋回はキーフレームモーション turn_l / turn_r の担当で、
                                   motion 側も /cmd_walk の angular.z を使わない
    パンチ 右/左    /cmd_motion "punch_r" / "punch_l"
    起き上がり      /cmd_motion "getup_front" / "getup_back"
    脱力            /estop true   … ラッチする。ウォッチドッグの発報先もここ
    ホームポジション /cmd_motion "home" → 少し置いて /estop false
    その場保持      /cmd_motion "hold" → 少し置いて /estop false … 今の姿勢のままトルクを入れる
                                   (転倒 → 脱力 → 起き上がり の経路。ホームは立位へ動き出すので使えない)
    自律動作        /autonomy true … behavior ノードに指令権を渡す

**脱力とトルクオンは /cmd_motion ではなく /estop に載せる。** 技名で送ると
「非常停止がラッチされているのに torque_on が届く」という矛盾した状態を motion 側で
解く羽目になる。トルクの ON/OFF は経路を 1 本に絞って、非常停止・ウォッチドッグ・
手動の脱力を全部そこへ集める。motion 側の約束は
「/estop true を受けたら即トルクOFF、false を受けたらトルクON」の 1 行で済む。

**ホームポジションは 2 段で送る。** まず /cmd_motion "home" で全軸の目標角を
ホームに置き、home_torque_delay 秒あけてから /estop false でトルクを入れる。
順番が逆になると、サーボに残っている古い目標角へ飛んでからホームへ動くことになる
(Feetech は目標角レジスタが生きたままトルクが入る)。1 tick で両方投げると motion 側の
受信順が保証されないので、わざと間を空けている。

自律動作 (/autonomy true) 中の teleop の振る舞い:

  * **/cmd_walk を出すのをやめる。** behavior と 2 重に publish すると指令が
    奪い合いになる (ros-architecture §2 の「同時に起動しない」運用を、起動したまま
    実現するのがこのトピック)
  * スティックとパンチは効かない。指令権は behavior にある
  * 下の 4 つだけは割り込みとして効き、**押した時点で自律動作を止める**:
      起き上がり / 脱力 / ホームポジション / その場保持

安全側の設計 (無線なので、ここが本体):

  * **デッドマン**: R1 を押している間しか /cmd_walk に値が乗らない。離せばゼロ。
    「押している間だけ動く」であって「押すと動き出す」ではない。
  * **足踏み**: R2 を押している間だけ /cmd_walk の linear.z に 1 を載せる (motion が
    その場で歩を踏む)。方向が要らないので R1 は見ない。これも「押している間だけ」で、
    離せば motion が停止シーケンスに入る (止まりきるまで約 3 秒)。
  * **無通信ウォッチドッグ**: /joy が joy_timeout 秒途切れたら脱力をラッチする。
    Bluetooth が切れる・電池が切れる・コントローラを踏む、はどれも実際に起きる。
    joy 側の autorepeat_rate を 0 より大きくしておくこと (launch で設定済み)。
    スティックを動かさない限り /joy が来ない設定だと、静止＝断線と見分けが付かない。
  * **起動時の再武装**: 起動直後・トルクオン直後・自律動作から戻った直後は、
    デッドマンを一度離すまで歩行指令を受け付けない。ボタンを押したままの状態で
    復帰していきなり歩き出す事故を防ぐ。
  * **加速度制限**: スティックの段差をそのまま速度指令にせず、1 周期あたりの
    変化量を制限する。二足で速度指令が階段状に飛ぶと、それだけで転ぶ。
  * **常時 20Hz 送信**: 止まっているときもゼロを送り続ける。無送信で「最後の指令が
    残る」より、ゼロが来続けるほうが motion 側の実装が単純で安全になる。
    (自律動作中だけは behavior が同じ役目を負う)

調整 (人が手で値を変える):

  項目の一覧・意味・単位・範囲は params.py の表が唯一の出どころで、
  config/ps5_dualsense.yaml はその表の全項目を並べたもの。値を変えたら
  `teleop_params --check <yaml>` で検査してから起動し直す。手順は docs/teleop_tuning.md。
"""

import math
import signal
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

from . import params as tunables
from .bindings import apply_deadzone, Binding, parse_motion_bindings

#: /estop と /autonomy は latched。teleop より後に motion・behavior を起動しても、
#: 直前の脱力状態・自律の可否が届く。ここを Volatile にすると「脱力させた状態で
#: motion を再起動したら動き出した」が起こりうる。
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
        # 項目の一覧・意味・単位・既定値・範囲は params.py の表に全部ある (人が触る値は
        # そこだけ)。範囲外は declare_all が起動時に弾く。
        tunables.declare_all(self)
        self._apply(tunables.read_all(self))

        # --- 状態 -----------------------------------------------------------
        self._joy = None            # 最新の Joy。まだ一度も来ていなければ None
        self._joy_stamp = None      # 最後に /joy を受けた時刻 (秒, ROS time)
        self._estop = False
        self._auto = False          # 自律動作中か
        self._armed = False         # デッドマンを一度離すまで False
        self._arm_since = {}        # 'home' / 'hold': ボタンを押し始めた時刻
        self._arm_fired = {}        # 同: 長押し成立済み。離すまで再発火させない
        self._auto_since = None     # 自律動作のボタンを押し始めた時刻
        self._auto_fired = False
        self._torque_at = None      # ホーム送信後、トルクを入れる時刻
        self._prev_motion = {}      # 技ボタンの前フレームの押下状態 (立ち上がり検出用)
        self._motion_until = 0.0    # 連射防止
        self._cmd = [0.0, 0.0]      # 実際に出している (x, y)。加速度制限後の値

        # --- 通信 -----------------------------------------------------------
        self._pub_walk = self.create_publisher(Twist, '/cmd_walk', 10)
        self._pub_estop = self.create_publisher(Bool, '/estop', LATCHED)
        self._pub_motion = self.create_publisher(String, '/cmd_motion', 10)
        self._pub_auto = self.create_publisher(Bool, '/autonomy', LATCHED)
        self.create_subscription(Joy, '/joy', self._on_joy, 10)

        # 起動直後に初期状態を 1 回流しておく。latched なので、後から起動した
        # motion・behavior はこれを受けて初期状態を確定できる。
        self._publish_estop()
        self._publish_auto()

        self._timer = self.create_timer(1.0 / self._rate_hz, self._tick)
        self.get_logger().info(
            f'teleop 起動。デッドマン={self._b_deadman.spec} 脱力={self._b_relax.spec} '
            f'ホーム={self._b_home.spec}({self._home_hold:.1f}s 長押し) '
            f'その場保持={self._b_hold.spec}({self._hold_hold:.1f}s 長押し) '
            f'足踏み={self._b_march.spec}(押している間) '
            f'自律={self._b_auto.spec}({self._auto_hold:.1f}s 長押し) '
            f'joy_timeout={self._joy_timeout:.2f}s')

    # ------------------------------------------------------------------ 時刻
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ パラメータ
    def _apply(self, cfg):
        """表 (params.py) から読んだ値を、この node の設定に写す。状態は触らない。"""
        self._rate_hz = cfg['rate_hz']
        self._joy_timeout = cfg['joy_timeout']
        self._deadzone = cfg['deadzone']
        self._ax = {k: cfg[f'axes.walk_{k}'] for k in ('x', 'y')}
        self._inv = {k: cfg[f'invert.walk_{k}'] for k in ('x', 'y')}
        self._scale = {k: cfg[f'scale.{k}'] for k in ('x', 'y')}
        self._accel = {k: cfg[f'accel.{k}'] for k in ('x', 'y')}
        self._b_deadman = Binding(cfg['buttons.deadman'])
        self._b_relax = Binding(cfg['buttons.relax'])
        self._b_home = Binding(cfg['buttons.home'])
        self._b_auto = Binding(cfg['buttons.autonomy'])
        self._b_hold = Binding(cfg['buttons.hold'])
        self._b_march = Binding(cfg['buttons.march'])
        self._home_hold = cfg['home_hold']
        self._home_motion = cfg['home_motion']
        self._home_delay = cfg['home_torque_delay']
        self._hold_hold = cfg['hold_hold']
        self._hold_motion = cfg['hold_motion']
        self._auto_hold = cfg['autonomy_hold']
        self._auto_stop_on_loss = cfg['autonomy.stop_on_joy_loss']
        self._motion_bindings = parse_motion_bindings(cfg['motion_bindings'])
        #: 割り込み扱いの技。デッドマン不要で、押すと自律動作を止める。
        self._motion_interrupts = set(cfg['motion_interrupts'])
        self._motion_needs_deadman = cfg['motion_requires_deadman']
        self._motion_cooldown = cfg['motion_cooldown']

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

        # 1) ウォッチドッグ。一度でも /joy が来たあとで途切れたら脱力。
        #    一度も来ていないうちは「まだ joy_node が上がっていない」なので待つ。
        if self._joy_stamp is not None and now - self._joy_stamp > self._joy_timeout:
            if self._auto and not self._auto_stop_on_loss:
                # 電波が切れても自律を続ける運用。非常停止の手が無くなるので、
                # 使うなら別の停止手段 (物理スイッチ) を用意してから。
                self._joy = None
            else:
                self._interrupt('無通信')
                self._set_estop(f'/joy が {now - self._joy_stamp:.2f}s 途切れた')
                self._joy = None

        target = [0.0, 0.0]
        march = False
        if self._joy is not None:
            axes, buttons = self._joy.axes, self._joy.buttons

            # 2) 割り込み群。自律動作中でも効き、押した時点で自律を止める。
            if self._b_relax.pressed(axes, buttons):
                self._interrupt('脱力')
                self._set_estop('脱力ボタン')
            self._handle_arm('home', self._b_home, self._home_hold, self._home_motion,
                             axes, buttons, now)
            self._handle_arm('hold', self._b_hold, self._hold_hold, self._hold_motion,
                             axes, buttons, now)

            # 3) 自律動作へ入る (長押し)。
            self._handle_autonomy(axes, buttons, now)

            # 4) 再武装。デッドマンを離した状態を一度見るまで指令を通さない。
            deadman = self._b_deadman.pressed(axes, buttons)
            if not deadman:
                if not self._armed and not self._estop and not self._auto:
                    self._armed = True
                    self.get_logger().info('デッドマン再武装。R1 で歩行指令が出せる')
            elif self._armed and not self._estop and not self._auto:
                # 左スティックの並行移動だけ。旋回は載せない (motion が使わない)。
                for i, k in enumerate(('x', 'y')):
                    target[i] = self._axis(axes, k) * self._scale[k]

            # 4') 足踏み。**押している間だけ**。方向が要らないのでデッドマン (R1) は
            #     見ない — このボタン自体が「離せば止まる」デッドマンになっている。
            #     脱力中・自律中・再武装前は通さない (歩行指令と同じ条件)。
            march = (self._armed and not self._estop and not self._auto
                     and self._b_march.pressed(axes, buttons))

            # 5) 技指令。押した瞬間だけ 1 回送る。
            self._handle_motion(axes, buttons, now, deadman)

        # 6) ホーム送信後のトルクオン (遅延実行)。
        self._handle_torque_on(now)

        # 7) 加速度制限を掛けて送信。脱力中もゼロを送り続ける (無送信にしない)。
        #    自律動作中だけは publish しない — behavior と奪い合いになるため。
        if self._auto:
            self._cmd = [0.0, 0.0]          # 戻ってきたときにゼロから始める
        else:
            dt = 1.0 / self._rate_hz
            for i, k in enumerate(('x', 'y')):
                self._cmd[i] = _slew(self._cmd[i], target[i], self._accel[k] * dt)
            msg = Twist()
            msg.linear.x, msg.linear.y = self._cmd
            # 足踏みは linear.z に載せる (motion ノードが > 0.5 で足踏みとみなす)。
            # /cmd_walk と同じ 1 本の指令なので、teleop が落ちたりコントローラが切れたり
            # すれば motion 側の cmd_timeout で足踏みも一緒に止まる。
            msg.linear.z = 1.0 if march else 0.0
            self._pub_walk.publish(msg)

    # ------------------------------------------------------------ 補助メソッド
    def _axis(self, axes, key) -> float:
        i = self._ax[key]
        v = axes[i] if 0 <= i < len(axes) else 0.0
        v = apply_deadzone(v, self._deadzone)
        return -v if self._inv[key] else v

    # ------------------------------------------------------------ 自律動作
    def _handle_autonomy(self, axes, buttons, now):
        """長押しで自律動作に入る。抜けるのは割り込み側 (_interrupt)。"""
        if not self._b_auto.pressed(axes, buttons):
            self._auto_since = None
            self._auto_fired = False
            return
        if self._auto or self._auto_fired:
            return              # 押しっぱなしで再発火させない (一度離すこと)
        if self._auto_since is None:
            self._auto_since = now
            return
        if now - self._auto_since < self._auto_hold:
            return
        self._auto_since = None
        self._auto_fired = True
        if self._estop:
            self.get_logger().warn('脱力中は自律動作に入らない。先にホームポジションへ')
            return
        self._auto = True
        self._armed = False
        self._cmd = [0.0, 0.0]
        self._pub_walk.publish(Twist())      # 最後に置いていく値をゼロにしてから黙る
        self._publish_auto()
        self.get_logger().warn(
            '*** 自律動作 開始 *** /cmd_walk は behavior に渡した。'
            '止めるには 起き上がり / 脱力 / ホーム / その場保持 のいずれか')

    def _interrupt(self, reason: str):
        """自律動作を止める。既に手動なら何もしない。"""
        if not self._auto:
            return
        self._auto = False
        self._auto_since = None
        self._armed = False          # 戻っても、デッドマンを一度離すまでは動かさない
        self._cmd = [0.0, 0.0]
        self._publish_auto()
        self.get_logger().warn(f'*** 自律動作 停止 *** 割り込み: {reason}')

    def _publish_auto(self):
        self._pub_auto.publish(Bool(data=self._auto))

    # ------------------------------------------------------- 脱力 / ホーム
    def _set_estop(self, reason: str):
        if self._estop:
            return
        self._estop = True
        self._armed = False
        self._cmd = [0.0, 0.0]        # 減速ではなく即ゼロ。脱力なので。
        self._arm_since.clear()
        self._torque_at = None        # 保留中のトルクオンは取り消す
        self._publish_estop()
        self.get_logger().error(f'*** 脱力 (トルクOFF) *** 理由: {reason}')

    def _handle_arm(self, key, binding, hold, motion, axes, buttons, now):
        """長押しで「技名を送ってからトルクを入れる」2 段送信。ホームとその場保持の共通部。

        key     'home' / 'hold'。押し始め時刻と発火済みフラグの置き場
        hold    長押し時間 [s]。誤って触ったときに急にトルクが入らないようにするため
        motion  先に /cmd_motion へ送る名前。home はホーム姿勢へ動き出し、hold は
                今の実測姿勢のまま固まる (motion ノード側の約束)

        脱力中かどうかに関わらず効く (脱力から復帰する手段でもある)。
        """
        if not binding.pressed(axes, buttons):
            self._arm_since.pop(key, None)
            self._arm_fired[key] = False
            return
        if self._arm_fired.get(key):
            # 押しっぱなしのまま長押し判定を繰り返すと連射される
            # (2026-08-28 実機で 1 秒ごとに再送されるのを確認)。一度離すこと。
            return
        since = self._arm_since.get(key)
        if since is None:
            self._arm_since[key] = now
            return
        if now - since < hold:
            return
        self._arm_since.pop(key, None)
        self._arm_fired[key] = True
        self._interrupt('ホームポジション' if key == 'home' else 'その場保持')
        self._pub_motion.publish(String(data=motion))
        self._torque_at = now + self._home_delay
        self.get_logger().info(
            f'/cmd_motion → {motion} ({self._home_delay:.2f}s 後にトルクオン)')

    def _handle_torque_on(self, now):
        """ホームの目標角が届いたころにトルクを入れる。"""
        if self._torque_at is None or now < self._torque_at:
            return
        self._torque_at = None
        self._armed = False           # トルクが入っても、デッドマンを離すまで動かさない
        if not self._estop:
            return                    # 既にトルクは入っている (ホーム姿勢だけ送った)
        self._estop = False
        self._publish_estop()
        self.get_logger().warn('トルクオン。デッドマンを一度離すと歩行指令を受け付ける')

    # ---------------------------------------------------------------- 技指令
    def _handle_motion(self, axes, buttons, now, deadman):
        for binding, name in self._motion_bindings:
            down = binding.pressed(axes, buttons)
            was = self._prev_motion.get(binding.spec, False)
            self._prev_motion[binding.spec] = down
            if not (down and not was):
                continue                      # 立ち上がりだけ拾う
            interrupt = name in self._motion_interrupts
            if self._auto and not interrupt:
                # 自律動作中の指令権は behavior にある。割り込み技だけが通る。
                self.get_logger().warn(f'自律動作中のため技 "{name}" は送らない')
                continue
            if self._estop:
                self.get_logger().warn(f'脱力中のため技 "{name}" は送らない')
                continue
            # 割り込み技 (起き上がりなど) はデッドマンを要求しない。転んだ機体を
            # 起こすのに R1 を押させると、自律動作からの割り込みが成立しない。
            if self._motion_needs_deadman and not deadman and not interrupt:
                self.get_logger().warn(f'デッドマン未押下のため技 "{name}" は送らない')
                continue
            if now < self._motion_until:
                continue
            self._motion_until = now + self._motion_cooldown
            if interrupt:
                self._interrupt(name)
            self._pub_motion.publish(String(data=name))
            self.get_logger().info(f'/cmd_motion → {name}')

    # ------------------------------------------------------------ 状態の送出
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
                self._auto = False
                self._publish_auto()          # 自律を残したまま消えない
                self._pub_walk.publish(Twist())
                self._estop = True
                self._publish_estop()
                time.sleep(0.05)
        except Exception:
            pass
        return super().destroy_node()


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
