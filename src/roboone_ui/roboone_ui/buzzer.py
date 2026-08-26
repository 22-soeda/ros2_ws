# -*- coding: utf-8 -*-
"""圧電サウンダー (村田 PKM13EPYH4000-A0) ドライバ。ROS 非依存。

Gate 3 で実機検証済み (2026-08-27): DC High は無音、4kHz で鳴動、3k/4k/5k のうち
4kHz が最も大きい (共振周波数の裏取り完了)。

★これは自励発振ブザーではない (ブリーフ §0.3 の最重要事項)
    外部駆動型なので DC をかけても鳴らない。4kHz 付近の矩形波で駆動して初めて音が出る。
    共振 4.0kHz から外れると音圧が急落するので、実質 4kHz のビープしか出せない。
    メロディ再生は設計に入れない。

配線 (回路リテイクで確定。変更禁止):
    BZ1 ← Q2 (2SC2712 NPN) ローサイド、R5=1kΩ ベース直列・R7=10kΩ プルダウン
    GPIO19 (PWM0 ch3)。GPIO High = ベース電流 → NPN ON の正論理。
"""

from .rp1_pwm import CH_BUZZER, open_channels

#: PKM13EPYH4000-A0 の共振周波数。ここを外すと音圧が急落する。
RESONANT_HZ = 4000

#: 矩形波のデューティ。50% が最も素直に鳴る。
DEFAULT_DUTY = 50


class Buzzer:
    """4kHz 固定のビープ。周波数を外から渡せるようにはしない (ブリーフ §1)。"""

    def __init__(self, freq_hz=RESONANT_HZ, duty=DEFAULT_DUTY, channel=CH_BUZZER):
        self._duty = duty
        _, chans = open_channels([channel], freq_hz)
        self._ch = chans[channel]
        self._on = False
        self.off()

    def on(self):
        if self._on:
            return
        self._ch.set_duty(self._duty)
        self._ch.enable()
        self._on = True

    def off(self):
        if self._on is False:
            # 初期化直後など、状態が不明なときも確実に落とす
            self._ch.off()
            return
        self._ch.off()
        self._on = False

    def close(self):
        """必ず止める。4kHz が鳴りっぱなしになると作業が続けられない (ブリーフ §1)。"""
        try:
            self._ch.off()
        finally:
            self._on = False
