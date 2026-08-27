# -*- coding: utf-8 -*-
"""joy_probe — どのボタン/軸が何番かを実機で確かめるための道具。

teleop の config に書く index は機種とドライバの組み合わせで変わる。推測で書くと
「非常停止だと思っていたボタンが実は旋回軸だった」が起きるので、必ずこれで確認する。

    ros2 run roboone_teleop joy_probe

ボタンを押すと ``button 10 ↓  (R1?)`` のように出る。スティックを倒すと
``axis 1 = -0.98  (左スティック 上下?)``。括弧の中は SDL GameController 標準配列
(= game_controller_node) を前提にした推測なので、joy_node を使うときは合わない。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

#: SDL GameController 標準配列 (game_controller_node が出す順) での呼び名。
SDL_BUTTONS = [
    '✕ cross', '○ circle', '□ square', '△ triangle',
    'Create', 'PS', 'Options', 'L3', 'R3', 'L1', 'R1',
    '十字 上', '十字 下', '十字 左', '十字 右',
    'マイク', 'パドル1', 'パドル2', 'パドル3', 'パドル4', 'タッチパッド',
]
SDL_AXES = [
    '左スティック 左右', '左スティック 上下',
    '右スティック 左右', '右スティック 上下',
    'L2', 'R2',
]

MOVE = 0.3   # これ以上動いた軸だけ表示する


class JoyProbe(Node):

    def __init__(self):
        super().__init__('joy_probe')
        self._buttons = []
        self._axes = []
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.get_logger().info(
            '/joy を待っています。ボタンを押すかスティックを倒してください (Ctrl-C で終了)')

    def _on_joy(self, msg):
        if len(self._buttons) != len(msg.buttons):
            self._buttons = [0] * len(msg.buttons)
            self._axes = list(msg.axes)
            self.get_logger().info(
                f'軸 {len(msg.axes)} 本 / ボタン {len(msg.buttons)} 個')

        for i, v in enumerate(msg.buttons):
            if v != self._buttons[i]:
                arrow = '↓ 押した' if v else '↑ 離した'
                print(f'  button {i:<2} {arrow}   config には "b{i}"   {_name(SDL_BUTTONS, i)}')
        self._buttons = list(msg.buttons)

        for i, v in enumerate(msg.axes):
            if abs(v - self._axes[i]) >= MOVE:
                sign = '+' if v > 0 else '-'
                print(f'  axis   {i:<2} = {v:+.2f}      config には "a{i}{sign}" '
                      f'  {_name(SDL_AXES, i)}')
                self._axes[i] = v


def _name(table, i):
    return f'({table[i]}?)' if i < len(table) else ''


def main(args=None):
    rclpy.init(args=args)
    node = JoyProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
