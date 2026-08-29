# -*- coding: utf-8 -*-
"""teleop ノード — PS5 (DualSense) コントローラで機体を無線操縦する。

ros-architecture §2/§4 の約束事:

    受け取る: /joy (sensor_msgs/Joy)          … joy パッケージのノードが出す
    出す:     /estop (std_msgs/Bool)          … 脱力 / トルクオン。イベント時
              /cmd_walk (geometry_msgs/Twist) … 歩行指令。20Hz
              /cmd_motion (std_msgs/String)   … 技名。イベント時
              /autonomy (std_msgs/Bool)       … 自律動作の可否。イベント時 (★新設)
              /ui/oled/text, /ui/led/pattern  … 操作者向けの状態表示
              /ui/buzzer                      … 状態が変わったときの合図

指令の対応 (割り当ての既定値は config/ps5_dualsense.yaml):

    並行移動        左スティック  → linear.x / linear.y。**angular.z は載せない**ので
                                   斜め前・斜め後ろへは機体の向きを変えずに歩く
    旋回            右スティック左右 → angular.z
    パンチ 右/左    /cmd_motion "punch_r" / "punch_l"
    起き上がり      /cmd_motion "getup_front" / "getup_back"
    脱力            /estop true   … ラッチする。ウォッチドッグの発報先もここ
    ホームポジション /cmd_motion "home" → 少し置いて /estop false
    その場保持      /cmd_motion "hold" → 少し置いて /estop false … 今の姿勢のままトルクを入れる
                                   (転倒 → 脱力 → 起き上がり の経路。ホームは立位へ動き出すので使えない)
    自律動作        /autonomy true … behavior ノードに指令権を渡す
    無線テスト      押している間だけブザーを鳴らし LED の色を変える

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
  * 下の 5 つだけは割り込みとして効き、**押した時点で自律動作を止める**:
      起き上がり / 脱力 / 無線確認ブザー / ホームポジション / その場保持

安全側の設計 (無線なので、ここが本体):

  * **デッドマン**: R1 を押している間しか /cmd_walk に値が乗らない。離せばゼロ。
    「押している間だけ動く」であって「押すと動き出す」ではない。
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

無線テストだけは上の制約の外にある。**デッドマン不要・脱力中でも動く**。
機体を安全な脱力状態に置いたまま、離れた場所で電波が届いているかを確かめるための
機能なので、動作条件を付けると用を成さない。

状態表示 (ui ノードへ):

  操作者からは「今どのモードか」が機体を見ても分からない。特に自律動作中は teleop が
  /cmd_walk を黙るので、behavior が起動していないと「入れたのに動かない」になり、
  故障と区別が付かない。そこで状態が変わるたびに OLED と LED へ出す。

      待機   NO LINK   dark    /joy がまだ来ていない
      脱力   RELAX     estop   赤の速い点滅
      手動   MANUAL    warn    デッドマンの再武装待ち (黄の点滅)
      手動   MANUAL    ready   武装済み。R1 で歩ける (緑の点灯)
      自律   AUTO      auto    青の点滅。人が近寄ってはいけない状態
      無線   LINK TEST link    シアンの点灯

  色は teleop 側に持たない。ui のプリセット名だけを送り、実際の色は ui が決める
  (roboone_ui の LED_PATTERNS)。1 箇所で色を管理できるのと、無線テストが終わった
  ときに「元の状態のプリセットをもう一度送る」だけで復帰できる利点がある。

調整 (人が手で値を変える):

  項目の一覧・意味・単位・範囲は params.py の表が唯一の出どころで、
  config/ps5_dualsense.yaml はその表の全項目を並べたもの。走らせたまま
  `ros2 param set /teleop scale.x 0.08` で試せて、決まったら YAML に写す。
  表にない名前を config に書くと起動時に警告が出る。手順は docs/teleop_tuning.md。
"""

import math
import os
import signal
import time

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from roboone_interfaces.msg import OledText
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

from . import params as tunables
from .bindings import apply_deadzone, Binding, parse_motion_bindings

#: /estop と /autonomy は latched。teleop より後に motion・behavior を起動しても、
#: 直前の脱力状態・自律の可否が届く。ここを Volatile にすると「脱力させた状態で
#: motion を再起動したら動き出した」が起こりうる。ui ノードの購読側も同じ設定なので、
#: /ui/* もこの profile で出す (VOLATILE な publisher は TRANSIENT_LOCAL な
#: subscriber とマッチしない)。
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
        # そこだけ)。ここでは表どおりに宣言して読むだけ。走らせたまま
        # `ros2 param set` で変えると _on_set_params が検査し、次の周期に
        # _reload_params が反映する (手順は docs/teleop_tuning.md)。
        tunables.declare_all(self)
        self._apply(tunables.read_all(self))
        self._params_dirty = False
        self.add_on_set_parameters_callback(self._on_set_params)
        self._warn_unknown_overrides()

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
        self._cmd = [0.0, 0.0, 0.0]  # 実際に出している (x, y, yaw)。加速度制限後の値
        self._estop_beat = 0
        self._warned_short = False
        self._link = False          # 無線テストのボタンを押しているか
        self._link_next = 0.0       # 次にブザーを鳴らし直す時刻
        self._ui_shown = None       # 最後に ui へ送った状態。変化時だけ送る

        # --- 通信 -----------------------------------------------------------
        self._pub_walk = self.create_publisher(Twist, '/cmd_walk', 10)
        self._pub_estop = self.create_publisher(Bool, '/estop', LATCHED)
        self._pub_motion = self.create_publisher(String, '/cmd_motion', 10)
        self._pub_auto = self.create_publisher(Bool, '/autonomy', LATCHED)
        # ui ノードの購読側が latched なので、こちらも latched で出す。
        # LED は /ui/led (直接 RGB 指定) ではなく /ui/led/pattern を使う。色を
        # teleop 側に持たないため (docstring「状態表示」参照)。
        self._pub_oled = self.create_publisher(OledText, '/ui/oled/text', LATCHED)
        self._pub_pattern = self.create_publisher(String, '/ui/led/pattern', LATCHED)
        self._pub_buzzer = self.create_publisher(String, '/ui/buzzer', LATCHED)
        self.create_subscription(Joy, '/joy', self._on_joy, 10)

        # 起動直後に初期状態を 1 回流しておく。latched なので、後から起動した
        # motion・behavior はこれを受けて初期状態を確定できる。
        self._publish_estop()
        self._publish_auto()
        self._publish_ui()

        self._timer = self.create_timer(1.0 / self._rate_hz, self._tick)
        self.get_logger().info(
            f'teleop 起動。デッドマン={self._b_deadman.spec} 脱力={self._b_relax.spec} '
            f'ホーム={self._b_home.spec}({self._home_hold:.1f}s 長押し) '
            f'その場保持={self._b_hold.spec}({self._hold_hold:.1f}s 長押し) '
            f'自律={self._b_auto.spec}({self._auto_hold:.1f}s 長押し) '
            f'無線テスト={self._b_link.spec} joy_timeout={self._joy_timeout:.2f}s')
        self._log_effective()
        self._check_motion_names()

    # ------------------------------------------------------------------ 時刻
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ パラメータ
    def _apply(self, cfg):
        """表 (params.py) から読んだ値を、この node の設定に写す。状態は触らない。"""
        self._rate_hz = cfg['rate_hz']
        self._joy_timeout = cfg['joy_timeout']
        self._deadzone = cfg['deadzone']
        self._ax = {k: cfg[f'axes.walk_{k}'] for k in ('x', 'y', 'yaw')}
        self._inv = {k: cfg[f'invert.walk_{k}'] for k in ('x', 'y', 'yaw')}
        self._scale = {k: cfg[f'scale.{k}'] for k in ('x', 'y', 'yaw')}
        self._accel = {k: cfg[f'accel.{k}'] for k in ('x', 'y', 'yaw')}
        self._b_deadman = Binding(cfg['buttons.deadman'])
        self._b_relax = Binding(cfg['buttons.relax'])
        self._b_home = Binding(cfg['buttons.home'])
        self._b_link = Binding(cfg['buttons.link_test'])
        self._b_auto = Binding(cfg['buttons.autonomy'])
        self._b_hold = Binding(cfg['buttons.hold'])
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
        self._motions_yaml = cfg['motions_yaml']
        # 無線テスト。joy_timeout (0.5s) では遅いので、数フレーム落ちた時点で鳴り止ませる。
        self._link_buzzer = cfg['link_test.buzzer']
        self._link_hz = cfg['link_test.buzzer_hz']
        self._link_stale = cfg['link_test.stale']
        # 状態表示。色は持たず ui のプリセット名だけを送る (docstring 参照)。
        self._ui_enable = cfg['ui.enable']
        self._ui_pattern = {k: cfg[f'ui.pattern.{k}']
                            for k in ('nolink', 'relax', 'unarmed', 'manual', 'auto', 'link')}
        self._ui_beep = {k: cfg[f'ui.buzzer.{k}'] for k in ('relax', 'auto', 'manual')}

    def _on_set_params(self, plist):
        """`ros2 param set` の検査。表の範囲・書式に外れた値は拒否して理由を返す。

        受けた値はここでは使わず、次の周期に _reload_params がまとめて読み直す
        (rclpy はこのコールバックの後で値を確定するので、ここで読むと古い値になる)。
        """
        known = tunables.by_name()
        for p in plist:
            t = known.get(p.name)
            if t is None:
                continue        # use_sim_time など、表にないものは関知しない
            err = tunables.validate(t, p.value)
            if err:
                self.get_logger().error(f'調整を拒否: {p.name} = {p.value!r} — {err}')
                return SetParametersResult(successful=False, reason=f'{p.name}: {err}')
            self.get_logger().info(f'調整: {p.name} = {p.value!r} (次の周期から効く)')
            self._params_dirty = True
        return SetParametersResult(successful=True)

    def _reload_params(self):
        """変更されたパラメータを読み直して反映する。周期が変わったらタイマーも作り直す。"""
        self._params_dirty = False
        old_rate = self._rate_hz
        self._apply(tunables.read_all(self))
        if self._rate_hz != old_rate:
            old = self._timer
            self._timer = self.create_timer(1.0 / self._rate_hz, self._tick)
            old.cancel()
        self._log_effective()
        self._check_motion_names()

    def _warn_unknown_overrides(self):
        """設定に表にない名前があれば起動時に言う。ROS は黙って捨てるので、ここで拾う。"""
        overrides = getattr(self, '_parameter_overrides', None) or {}
        for name in tunables.unknown_keys(overrides.keys()):
            hint = tunables.suggest(name)
            self.get_logger().warn(
                f'config の "{name}" は teleop に無い項目なので効いていない'
                + (f' ("{hint}" の打ち間違い?)' if hint else '')
                + '。一覧: ros2 run roboone_teleop teleop_params')

    def _log_effective(self):
        """いま効いている調整値を 1 行で出す。"""
        s, a = self._scale, self._accel
        self.get_logger().info(
            '調整値: scale x/y/yaw=%.3f/%.3f/%.2f accel=%.2f/%.2f/%.2f deadzone=%.2f '
            'joy_timeout=%.2fs rate=%.0fHz hold home/auto=%.1f/%.1fs' % (
                s['x'], s['y'], s['yaw'], a['x'], a['y'], a['yaw'], self._deadzone,
                self._joy_timeout, self._rate_hz, self._home_hold, self._auto_hold))

    # ------------------------------------------------------------ 技名の照合
    def _check_motion_names(self):
        """割り当てた技名が motions.yaml に本当にあるかを照合し、無ければ警告する。

        motion ノードは /cmd_motion を受けた瞬間に「知らない技」と言うだけなので、
        ここで言わないと試合中にボタンを押すまで分からない
        (docs/無線操縦_不足項目レビュー.md §3.1)。
        """
        path = self._motions_yaml or self._default_motions_yaml()
        if not path or not os.path.exists(path):
            self.get_logger().info(
                f'motions.yaml が見つからないので技名の照合は飛ばす (motions_yaml={path!r})')
            return
        try:
            defined = tunables.load_motion_names(path)
        except Exception as e:      # 読めない理由をそのまま見せる
            self.get_logger().warn(f'motions.yaml を読めないので技名の照合は飛ばす ({path}): {e}')
            return
        used = [name for _, name in self._motion_bindings]
        missing = tunables.missing_motions(
            used, defined, builtin=(self._home_motion, self._hold_motion))
        for name in missing:
            self.get_logger().warn(
                f'技 "{name}" は motions.yaml に無い。割り当てたボタンを押しても何も起きない ({path})')
        if not missing:
            self.get_logger().info(f'技名の照合 OK ({len(used)} 件、{path})')

    def _default_motions_yaml(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('roboone_motion_node')
        except Exception:           # パッケージが無い環境 (teleop 単体) では照合しない
            return ''
        return os.path.join(share, 'config', 'motions.yaml')

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
        if self._params_dirty:
            self._reload_params()
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

        target = [0.0, 0.0, 0.0]
        link = False
        if self._joy is not None:
            axes, buttons = self._joy.axes, self._joy.buttons
            self._check_length(axes, buttons)

            # 2) 割り込み群。自律動作中でも効き、押した時点で自律を止める。
            if self._b_relax.pressed(axes, buttons):
                self._interrupt('脱力')
                self._set_estop('脱力ボタン')
            self._handle_arm('home', self._b_home, self._home_hold, self._home_motion,
                             axes, buttons, now)
            self._handle_arm('hold', self._b_hold, self._hold_hold, self._hold_motion,
                             axes, buttons, now)
            link = (self._b_link.pressed(axes, buttons)
                    and now - self._joy_stamp <= self._link_stale)

            # 3) 自律動作へ入る (長押し)。
            self._handle_autonomy(axes, buttons, now)

            # 4) 再武装。デッドマンを離した状態を一度見るまで指令を通さない。
            deadman = self._b_deadman.pressed(axes, buttons)
            if not deadman:
                if not self._armed and not self._estop and not self._auto:
                    self._armed = True
                    self.get_logger().info('デッドマン再武装。R1 で歩行指令が出せる')
            elif self._armed and not self._estop and not self._auto:
                # 左スティックは linear だけ、右スティックは angular だけ。混ぜない。
                # 斜めに歩くときに機体が勝手に旋回すると、狙った方向へ進まなくなる。
                for i, k in enumerate(('x', 'y', 'yaw')):
                    target[i] = self._axis(axes, k) * self._scale[k]

            # 5) 技指令。押した瞬間だけ 1 回送る。
            self._handle_motion(axes, buttons, now, deadman)

        # 5.5) 無線テスト。デッドマンも要らず、脱力中でも動く (docstring 参照)。
        #      /joy が途切れたら link=False になり、鳴り止む。
        self._handle_link(link, now)

        # 6) ホーム送信後のトルクオン (遅延実行)。
        self._handle_torque_on(now)

        # 7) 加速度制限を掛けて送信。脱力中もゼロを送り続ける (無送信にしない)。
        #    自律動作中だけは publish しない — behavior と奪い合いになるため。
        if self._auto:
            self._cmd = [0.0, 0.0, 0.0]     # 戻ってきたときにゼロから始める
        else:
            dt = 1.0 / self._rate_hz
            for i, k in enumerate(('x', 'y', 'yaw')):
                self._cmd[i] = _slew(self._cmd[i], target[i], self._accel[k] * dt)
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = self._cmd
            self._pub_walk.publish(msg)

        # 7.5) 状態表示。変化したときだけ ui へ送る。
        self._publish_ui()

        # 8) 脱力中は 2 秒に 1 回だけ再送。latched QoS があるので本来不要だが、
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
        if self._warned_short:
            return
        need_ax = max(self._ax.values())
        need_bt = max((b.index for b, _ in self._motion_bindings), default=0)
        need_bt = max([need_bt] + [b.index for b in
                                   (self._b_deadman, self._b_relax, self._b_home,
                                    self._b_link, self._b_auto, self._b_hold) if not b.is_axis])
        short = []
        if need_ax >= len(axes):
            short.append(f'軸 {len(axes)} 本 < index {need_ax}')
        if need_bt >= len(buttons):
            short.append(f'ボタン {len(buttons)} 個 < index {need_bt}')
        if short:
            self._warned_short = True
            self.get_logger().warn(
                f'Joy が足りない ({" / ".join(short)})。config の割り当てが機種に'
                '合っていない可能性が高い (joy_probe で確認すること)')

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
        self._cmd = [0.0, 0.0, 0.0]
        self._pub_walk.publish(Twist())      # 最後に置いていく値をゼロにしてから黙る
        self._publish_auto()
        self.get_logger().warn(
            '*** 自律動作 開始 *** /cmd_walk は behavior に渡した。'
            '止めるには 起き上がり / 脱力 / 無線確認 / ホーム / その場保持 のいずれか')

    def _interrupt(self, reason: str):
        """自律動作を止める。既に手動なら何もしない。"""
        if not self._auto:
            return
        self._auto = False
        self._auto_since = None
        self._armed = False          # 戻っても、デッドマンを一度離すまでは動かさない
        self._cmd = [0.0, 0.0, 0.0]
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
        self._cmd = [0.0, 0.0, 0.0]   # 減速ではなく即ゼロ。脱力なので。
        self._arm_since.clear()
        self._torque_at = None        # 保留中のトルクオンは取り消す
        self._estop_beat = 0
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

    # ------------------------------------------------------------ 無線テスト
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
            self._interrupt('無線確認ブザー')
            self.get_logger().info('無線テスト 開始 (電波が届いている)')
        elif not down and self._link:
            self._link = False
            self.get_logger().info('無線テスト 終了')
        # LED/OLED は _publish_ui() が状態から決める。離したときの復帰も、
        # 元の状態のプリセットをもう一度送るだけで済む。

        if self._link and now >= self._link_next:
            self._link_next = now + 1.0 / self._link_hz
            self._pub_buzzer.publish(String(data=self._link_buzzer))

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

    # ------------------------------------------------------------ 状態表示
    def _ui_state(self):
        """今の状態を (キー, OLED 1 行目, OLED 2 行目, 文字色) で返す。

        OLED は 8x8 フォントで 1 行 12 文字ちょうど (roboone_ui 実測)。
        はみ出しは ui 側がクリップするが、読めなくなるので 12 文字に収める。
        日本語は出せないので ASCII で書く。
        """
        if self._link:
            return ('link', 'LINK TEST', 'radio ok', (0, 200, 255))
        if self._joy_stamp is None:
            return ('nolink', 'TELEOP', 'no joy', (120, 120, 120))
        if self._estop:
            return ('relax', 'RELAX', 'OPTIONS=home', (255, 60, 60))
        if self._auto:
            return ('auto', 'AUTO', 'any 5 = stop', (60, 140, 255))
        if not self._armed:
            return ('unarmed', 'MANUAL', 'release R1', (255, 180, 0))
        return ('manual', 'MANUAL', 'R1 = walk', (0, 255, 0))

    def _publish_ui(self):
        """状態が変わったときだけ ui へ送る。

        毎周期送っても ui 側はタプル比較でデバウンスするが、20Hz で latched
        トピックを叩き続ける意味は無いのでこちらで止める。
        """
        if not self._ui_enable:
            return
        state = self._ui_state()
        if state == self._ui_shown:
            return
        key, line1, line2, color = state
        prev = self._ui_shown[0] if self._ui_shown else None
        self._ui_shown = state

        msg = OledText()
        msg.line1, msg.line2 = line1, line2
        msg.r, msg.g, msg.b = color
        self._pub_oled.publish(msg)
        self._pub_pattern.publish(String(data=self._ui_pattern[key]))

        # 音は「操作者が見ていなくても気付くべき変化」だけ。無線テストは
        # それ自体がブザーを鳴らしているので鳴らさない。
        if key != prev and key in self._ui_beep and prev is not None:
            self._pub_buzzer.publish(String(data=self._ui_beep[key]))

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
                self._link = False
                self._publish_ui()            # 消灯ではなく「脱力」を表示して終わる
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
