# -*- coding: utf-8 -*-
"""teleop ノードの結線テスト。コントローラ無しで安全側の挙動を確かめる。

実機が無くても壊れたことに気付けるようにしておく。特にデッドマンと
ウォッチドッグは、壊れても手元では「動かないだけ」に見えてしまい、
気付くのが実機で歩かせたときになるので自動で見張る。
"""

import threading
import time

from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from roboone_interfaces.msg import LedColor
from roboone_teleop.bindings import apply_deadzone, Binding, parse_motion_bindings
from roboone_teleop.teleop_node import LATCHED, TeleopNode
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

# テスト用のパラメータ。本番 config と同じキーだが、待ち時間を詰めてある。
PARAMS = {
    'rate_hz': 50.0, 'joy_timeout': 0.3, 'deadzone': 0.1,
    'axes.walk_x': 1, 'axes.walk_y': 0, 'axes.walk_yaw': 2,
    'invert.walk_x': True, 'invert.walk_y': True, 'invert.walk_yaw': True,
    'scale.x': 1.0, 'scale.y': 1.0, 'scale.yaw': 1.0,
    'accel.x': 1000.0, 'accel.y': 1000.0, 'accel.yaw': 1000.0,
    'buttons.deadman': 'b10', 'buttons.relax': 'b9',
    'buttons.torque_on': 'b6', 'buttons.link_test': 'b4',
    'torque_on_hold': 0.3,
    'motion_bindings': ['b1:punch_r'], 'motion_requires_deadman': True,
    'motion_cooldown': 0.05,
    'link_test.color': [0, 200, 255], 'link_test.release_color': [0, 0, 0],
    'link_test.buzzer': 'beep', 'link_test.buzzer_hz': 15.0,
    'link_test.stale': 0.1,
}

DEADMAN, RELAX, TORQUE_ON, LINK, PUNCH_R = 10, 9, 6, 4, 1

N_AXES = 6
N_BUTTONS = 21          # DualSense をひととおり覆う数


def _overrides():
    return [Parameter(k, value=v) for k, v in PARAMS.items()]


class Harness(Node):
    """Joy を撃ち込んで、出てきた指令を溜める側。"""

    def __init__(self):
        super().__init__('teleop_test_harness')
        self.walk = []
        self.estop = []
        self.motion = []
        self.led = []
        self.buzzer = []
        self._joy = self.create_publisher(Joy, '/joy', 10)
        self.create_subscription(Twist, '/cmd_walk', lambda m: self.walk.append(m), 10)
        self.create_subscription(Bool, '/estop', lambda m: self.estop.append(m.data), LATCHED)
        self.create_subscription(String, '/cmd_motion', lambda m: self.motion.append(m.data), 10)
        self.create_subscription(
            LedColor, '/ui/led', lambda m: self.led.append((m.r, m.g, m.b)), LATCHED)
        self.create_subscription(
            String, '/ui/buzzer', lambda m: self.buzzer.append(m.data), LATCHED)

    def send(self, axes=None, buttons=None):
        msg = Joy()
        msg.axes = list(axes or [0.0] * N_AXES)
        msg.buttons = list(buttons or [0] * N_BUTTONS)
        self._joy.publish(msg)


@pytest.fixture
def rig():
    rclpy.init()
    teleop = TeleopNode(parameter_overrides=_overrides())
    harness = Harness()
    ex = SingleThreadedExecutor()
    ex.add_node(teleop)
    ex.add_node(harness)
    stop = threading.Event()
    t = threading.Thread(target=lambda: _spin(ex, stop), daemon=True)
    t.start()
    time.sleep(0.3)                      # 接続確立を待つ
    try:
        yield harness, teleop
    finally:
        stop.set()
        t.join(timeout=2.0)
        harness.destroy_node()
        teleop.destroy_node()
        rclpy.shutdown()


def _spin(ex, stop):
    while not stop.is_set():
        ex.spin_once(timeout_sec=0.02)


def _pump(harness, axes=None, buttons=None, seconds=0.4, hz=50.0):
    """/joy を撃ち続ける。実機の autorepeat_rate に相当。"""
    end = time.time() + seconds
    while time.time() < end:
        harness.send(axes, buttons)
        time.sleep(1.0 / hz)


# ---------------------------------------------------------------- bindings
def test_binding_button():
    b = Binding('b5')
    assert b.pressed([], [0, 0, 0, 0, 0, 1])
    assert not b.pressed([], [0, 0, 0, 0, 0, 0])
    assert not b.pressed([], [0, 0])           # 短い Joy でも落ちない


def test_binding_axis_sign():
    up = Binding('a7-')
    assert up.pressed([0] * 7 + [-1.0], [])
    assert not up.pressed([0] * 7 + [1.0], [])


def test_binding_rejects_garbage():
    for bad in ('', 'x1', 'a7', 'b-1'):
        with pytest.raises(ValueError):
            Binding(bad)


def test_motion_bindings_parse():
    assert parse_motion_bindings(['b0:home'])[0][1] == 'home'
    with pytest.raises(ValueError):
        parse_motion_bindings(['b0'])


def test_deadzone_is_continuous():
    assert apply_deadzone(0.09, 0.1) == 0.0
    assert apply_deadzone(0.11, 0.1) == pytest.approx(0.0111, abs=1e-3)   # 段差が無い
    assert apply_deadzone(-1.0, 0.1) == pytest.approx(-1.0)


# ------------------------------------------------------------------ ノード
def test_publishes_zero_without_joy(rig):
    """/joy が一度も来ていなくても、ゼロ指令は出続ける。"""
    harness, _ = rig
    time.sleep(0.4)
    assert len(harness.walk) >= 3
    assert all(m.linear.x == 0.0 for m in harness.walk)


def test_deadman_gates_walk(rig):
    """デッドマンを押していない間はスティックを倒しても値が乗らない。"""
    harness, teleop = rig
    axes = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0]      # 左スティック 前倒し
    _pump(harness, axes, None, seconds=0.4)     # デッドマン無し
    assert all(m.linear.x == 0.0 for m in harness.walk)

    harness.walk.clear()
    buttons = [0] * N_BUTTONS
    buttons[DEADMAN] = 1
    _pump(harness, axes, buttons, seconds=0.4)
    assert max(m.linear.x for m in harness.walk) > 0.0
    assert not teleop._estop


def test_deadman_held_at_startup_is_ignored(rig):
    """起動時からデッドマンを押しっぱなしなら、一度離すまで動かさない。"""
    harness, _ = rig
    axes = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0]
    buttons = [0] * N_BUTTONS
    buttons[DEADMAN] = 1
    _pump(harness, axes, buttons, seconds=0.5)  # 最初から押しっぱなし
    assert all(m.linear.x == 0.0 for m in harness.walk)


def test_relax_latches_and_torque_on_needs_long_press(rig):
    harness, teleop = rig
    _pump(harness, seconds=0.2)                 # まず再武装させる

    buttons = [0] * N_BUTTONS
    buttons[RELAX] = 1                          # L1 脱力
    _pump(harness, None, buttons, seconds=0.2)
    assert teleop._estop
    assert harness.estop[-1] is True

    # トルクオンを短く押しても入らない
    buttons = [0] * N_BUTTONS
    buttons[TORQUE_ON] = 1
    _pump(harness, None, buttons, seconds=0.15)
    assert teleop._estop

    # 長押しで入る
    _pump(harness, None, buttons, seconds=0.5)
    assert not teleop._estop
    assert harness.estop[-1] is False


def test_watchdog_trips_on_joy_loss(rig):
    """/joy が途切れたら非常停止をラッチする。無線での本命。"""
    harness, teleop = rig
    _pump(harness, seconds=0.3)
    assert not teleop._estop
    time.sleep(0.6)                             # joy_timeout=0.3 を超えて黙る
    assert teleop._estop
    assert harness.estop[-1] is True


def test_motion_requires_deadman(rig):
    harness, _ = rig
    _pump(harness, seconds=0.2)

    buttons = [0] * N_BUTTONS
    buttons[PUNCH_R] = 1                        # ○ だけ
    _pump(harness, None, buttons, seconds=0.2)
    assert harness.motion == []

    _pump(harness, seconds=0.1)                 # 一度離す
    buttons[DEADMAN] = 1                        # ○ + R1
    _pump(harness, None, buttons, seconds=0.2)
    assert harness.motion == ['punch_r']        # 立ち上がり 1 回だけ


def test_translation_does_not_rotate(rig):
    """左スティックだけを斜めに倒しても angular.z は 0 のまま。"""
    harness, _ = rig
    _pump(harness, seconds=0.2)                 # 再武装
    harness.walk.clear()
    buttons = [0] * N_BUTTONS
    buttons[DEADMAN] = 1
    axes = [-1.0, -1.0, 0.0, 0.0, 0.0, 0.0]     # 左スティックを斜め前左へ
    _pump(harness, axes, buttons, seconds=0.4)
    moved = [m for m in harness.walk if m.linear.x != 0.0]
    assert moved, '斜め入力で前後成分が出ていない'
    assert all(m.linear.y > 0.0 for m in moved), '左倒しで +y が出ていない'
    assert all(m.angular.z == 0.0 for m in harness.walk), '並行移動で旋回が混ざった'


def test_yaw_comes_only_from_right_stick(rig):
    harness, _ = rig
    _pump(harness, seconds=0.2)
    harness.walk.clear()
    buttons = [0] * N_BUTTONS
    buttons[DEADMAN] = 1
    axes = [0.0, 0.0, -1.0, 0.0, 0.0, 0.0]      # 右スティックを左へ
    _pump(harness, axes, buttons, seconds=0.4)
    turning = [m for m in harness.walk if m.angular.z != 0.0]
    assert turning, '右スティックで旋回指令が出ていない'
    assert all(m.angular.z > 0.0 for m in turning), '左旋回が +z になっていない'
    assert all(m.linear.x == 0.0 and m.linear.y == 0.0 for m in harness.walk)


def test_link_test_needs_no_deadman_and_works_while_relaxed(rig):
    """無線テストはデッドマン不要・脱力中でも動く。動作条件を付けると用を成さない。"""
    harness, teleop = rig
    _pump(harness, seconds=0.2)

    buttons = [0] * N_BUTTONS
    buttons[RELAX] = 1                          # 先に脱力させておく
    _pump(harness, None, buttons, seconds=0.15)
    assert teleop._estop
    harness.led.clear()
    harness.buzzer.clear()

    buttons = [0] * N_BUTTONS
    buttons[LINK] = 1                           # デッドマンは押さない
    _pump(harness, None, buttons, seconds=0.5)
    assert harness.led[0] == (0, 200, 255), 'LED の色が変わっていない'
    assert len(harness.buzzer) >= 4, f'ブザーの再送が少なすぎる: {harness.buzzer}'
    assert set(harness.buzzer) == {'beep'}

    n_before = len(harness.buzzer)
    _pump(harness, None, None, seconds=0.3)     # 離す
    assert harness.led[-1] == (0, 0, 0), 'LED が消えていない'
    assert len(harness.buzzer) == n_before, '離したのにブザーが鳴り続けている'


def test_link_test_stops_when_joy_is_lost(rig):
    """電波が切れたら鳴りっぱなしにしない。

    ここは joy_timeout (脱力までの 0.3s) より速く止まること。無線テストは
    「どこまで電波が届くか」を耳で探す機能なので、切れた瞬間に鳴り止まないと
    範囲の境目が分からない。
    """
    harness, _ = rig
    buttons = [0] * N_BUTTONS
    buttons[LINK] = 1
    _pump(harness, None, buttons, seconds=0.3)
    assert harness.led[-1] == (0, 200, 255)

    time.sleep(0.2)                             # link_test.stale=0.1 を超えて黙る
    assert harness.led[-1] == (0, 0, 0), '電波が切れても LED が点いたまま'
    n_after_drop = len(harness.buzzer)
    time.sleep(0.3)
    assert len(harness.buzzer) == n_after_drop, '電波が切れてもブザーが鳴り続けている'
