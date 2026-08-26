# -*- coding: utf-8 -*-
"""フルカラー LED (OptoSupply OSTBMAZ2C1D) ドライバ。ROS 非依存。

Gate 2 で実機検証済み (2026-08-27): pwm0=赤 / pwm1=緑 / pwm2=青、4kHz でちらつきなし。

★極性 (ブリーフ §0.2 の最重要事項)
    コモンアノード (+5V 直結) だが、GPIO とカソードの間に Nch MOSFET (2N7002K) が
    ローサイドで入っている。したがって **GPIO High = MOSFET ON = 点灯** の正論理。
    デューティ 100% = 最大輝度。パターンマッチで反転させないこと。

配線 (回路リテイクで確定。変更禁止):
    J9-4 Red   ← R3 =150Ω ← Q3 ← GPIO12 (PWM0 ch0)
    J9-3 Green ← R13=100Ω ← Q4 ← GPIO13 (PWM0 ch1)
    J9-2 Blue  ← R16=100Ω ← Q5 ← GPIO18 (PWM0 ch2)

★3 色は同一デューティでは同じ明るさにならない
    20mA 時の光度 緑4000 / 赤2300 / 青1200 mcd、抵抗も 赤150Ω・緑青100Ω と違う。
    GAIN は輝度を揃える係数であって「白に見える混色」ではない。白点は目で詰める。
"""

from .rp1_pwm import CH_BLUE, CH_GREEN, CH_RED, open_channels

#: PWM 周波数。IEEE 1789-2015 で PWM 調光は 3kHz 超なら生体影響の観点で無制限。
#: ブザー (ch3) と同じ 4kHz に揃えてある。RP1 の周期がチャンネル間で共有だった
#: 場合でも破綻しないようにするための意図的な保険 (ブリーフ §0.4)。
DEFAULT_FREQ_HZ = 4000

#: 輝度を揃える係数。ブリーフ §0.2 の出発点の値。実機の目視で詰めて確定する。
DEFAULT_GAIN = {'r': 0.52, 'g': 0.30, 'b': 1.00}


class RgbLed:
    """RGB 3 チャンネルをまとめて扱う。指示値 0-255 を GAIN 付きデューティに落とす。"""

    def __init__(self, freq_hz=DEFAULT_FREQ_HZ, gain=None,
                 ch_r=CH_RED, ch_g=CH_GREEN, ch_b=CH_BLUE):
        self.gain = dict(gain or DEFAULT_GAIN)
        self._map = {'r': ch_r, 'g': ch_g, 'b': ch_b}
        _, self._ch = open_channels([ch_r, ch_g, ch_b], freq_hz)
        self._last = None
        self.set_rgb(0, 0, 0)

    def set_rgb(self, r, g, b):
        """r/g/b は 0-255 の指示値。GAIN を掛けてデューティ % にする。"""
        vals = {'r': r, 'g': g, 'b': b}
        if vals == self._last:
            return
        for key, v in vals.items():
            duty = (max(0, min(255, int(v))) / 255.0) * 100.0 * self.gain[key]
            ch = self._ch[self._map[key]]
            ch.set_duty(duty)
            if duty > 0:
                ch.enable()
            else:
                ch.disable()
        self._last = vals

    def off(self):
        self.set_rgb(0, 0, 0)

    def close(self):
        """全チャンネル消灯・無効化。終了時に必ず呼ぶこと (ブリーフ §1)。"""
        for ch in self._ch.values():
            ch.off()
        self._last = None
