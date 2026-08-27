# -*- coding: utf-8 -*-
"""ボタン/軸の割り当てを表す小道具。

teleop ノード本体から切り離してあるのは、割り当ての解釈だけを単体テストしたいから。
ROS には一切依存しない (sensor_msgs/Joy の中身と同じ形の list を受けるだけ)。
"""


class Binding:
    """「どの入力が押されたか」を 1 つ表す。

    表記は文字列 1 つ。config/*.yaml と `ros2 param set` の両方で書けるように、
    ネストした dict ではなく平たい文字列にしてある。

      ``b5``   … buttons[5] が 1 のとき ON
      ``a7+``  … axes[7] が +しきい値 を超えたとき ON
      ``a7-``  … axes[7] が -しきい値 を下回ったとき ON

    ``a`` 形式があるのは十字キー対策。十字キーは joy_node (生の SDL Joystick) では
    hat = 軸として出るが、game_controller_node (SDL GameController) ではボタンとして
    出る。どちらのノードを使っても同じ config で書けるようにする。
    """

    THRESHOLD = 0.5

    def __init__(self, spec: str):
        self.spec = spec = str(spec).strip()
        if len(spec) < 2 or spec[0] not in 'ab':
            raise ValueError(f'割り当ての書式が不正: {spec!r} (例: "b5", "a7-")')
        self.is_axis = spec[0] == 'a'
        if self.is_axis:
            if spec[-1] not in '+-':
                raise ValueError(f'軸の割り当ては符号で終わること: {spec!r} (例: "a7-")')
            self.sign = 1.0 if spec[-1] == '+' else -1.0
            self.index = int(spec[1:-1])
        else:
            self.sign = 0.0
            self.index = int(spec[1:])
        if self.index < 0:
            raise ValueError(f'index が負: {spec!r}')

    def pressed(self, axes, buttons) -> bool:
        """今このフレームで ON かどうか。範囲外の index は「押されていない」扱い。"""
        if self.is_axis:
            if self.index >= len(axes):
                return False
            return axes[self.index] * self.sign > self.THRESHOLD
        if self.index >= len(buttons):
            return False
        return bool(buttons[self.index])

    def __repr__(self):
        return f'Binding({self.spec!r})'


def parse_motion_bindings(specs):
    """``["b0:home", "a7-:kick_r"]`` を [(Binding, 技名), ...] にする。"""
    out = []
    for item in specs:
        item = str(item)
        if ':' not in item:
            raise ValueError(f'motion_bindings は "<割り当て>:<技名>" 形式: {item!r}')
        spec, name = item.split(':', 1)
        name = name.strip()
        if not name:
            raise ValueError(f'技名が空: {item!r}')
        out.append((Binding(spec), name))
    return out


def apply_deadzone(value: float, deadzone: float) -> float:
    """不感帯を切ったうえで、残りを 0..1 に引き伸ばす。

    単純に「不感帯以下を 0 にする」だけだと、指を少し傾けた瞬間に deadzone ぶんの
    段差で指令が飛ぶ。歩行指令にいきなり段が入るのは避けたいので線形に伸ばす。
    """
    if deadzone >= 1.0:
        return 0.0
    a = abs(value)
    if a <= deadzone:
        return 0.0
    scaled = (a - deadzone) / (1.0 - deadzone)
    return scaled if value > 0.0 else -scaled
