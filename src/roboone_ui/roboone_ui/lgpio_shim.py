# -*- coding: utf-8 -*-
"""luma.core に渡す RPi.GPIO 互換 GPIO バックエンド (lgpio 実装)。

なぜ必要か
----------
apt の python3-luma.core 2.4.1 は GPIO バックエンドとして RPi.GPIO しか持たない
(/usr/lib/python3/dist-packages/luma/core/lib.py の __rpi_gpio__ が唯一の実装)。
RPi.GPIO は Pi 5 (RP1) では動かない。さらに Ubuntu 24.04 では /dev/gpiomem* が
root 専用なので、メモリマップ方式のライブラリはそもそも権限が無い。

luma が実際に呼ぶ API は下の 5 つだけ (serial.py の bitbang / gpio_cs_spi を読んで確認):
    LOW / HIGH / OUT  定数
    setup(pin, direction)             … bitbang._configure
    setup(pin, direction, initial=..) … gpio_cs_spi.__init__
    output(pin, value)                … value は 0/1 とは限らない (byte & 0x80 が来る)
    cleanup(pin)  /  cleanup([pins])

そこを lgpio の文字デバイス方式 (/dev/gpiochip4 = pinctrl-rp1) で埋める。
gpiochip4 は udev ルール /lib/udev/rules.d/60-gpio.rules により root:dialout 0660 で、
dialout 所属の一般ユーザーから開ける (2026-08-27 実機確認済み)。
"""

import lgpio

#: RP1 (GPIOヘッダ) の gpiochip 番号。ライン番号 = BCM GPIO 番号。
RP1_GPIOCHIP = 4


class LgpioBackend:
    """RPi.GPIO 互換の最小 API を lgpio で実装したもの。"""

    # luma が参照する定数
    LOW = 0
    HIGH = 1
    OUT = 1
    IN = 0
    BCM = 11      # luma は setmode を呼ばないが、念のため属性だけ持たせる
    BOARD = 10

    def __init__(self, chip=RP1_GPIOCHIP):
        self._chip_num = chip
        self._h = lgpio.gpiochip_open(chip)
        self._claimed = set()

    # --- RPi.GPIO 互換 API -------------------------------------------------
    def setmode(self, mode):
        """RPi.GPIO 互換のためだけの no-op。lgpio は常に BCM 番号。"""

    def setwarnings(self, flag):
        """no-op。"""

    def setup(self, pin, direction, initial=None):
        if direction != self.OUT:
            raise NotImplementedError(
                'このシムは出力専用。入力が要るようになったら gpio_claim_input を足すこと')
        level = 1 if (initial is not None and initial) else 0
        lgpio.gpio_claim_output(self._h, int(pin), level)
        self._claimed.add(int(pin))

    def output(self, pin, value):
        # luma の bitbang は (byte & 0x80) のような非 0/1 の int を渡してくる
        lgpio.gpio_write(self._h, int(pin), 1 if value else 0)

    def input(self, pin):  # noqa: A003  RPi.GPIO 互換のため名前を変えられない
        return lgpio.gpio_read(self._h, int(pin))

    def cleanup(self, pins=None):
        if pins is None:
            targets = list(self._claimed)
        elif isinstance(pins, (list, tuple, set)):
            targets = [int(p) for p in pins]
        else:
            targets = [int(pins)]
        for p in targets:
            if p in self._claimed:
                try:
                    lgpio.gpio_write(self._h, p, 0)   # 消灯側に倒してから解放
                    lgpio.gpio_free(self._h, p)
                except Exception:
                    pass
                self._claimed.discard(p)

    # --- 追加ヘルパ --------------------------------------------------------
    def hold_low(self, pin):
        """ピンを Low 出力にして保持する (OLED の ソフト CS を張りっぱなしにする用)。"""
        self.setup(pin, self.OUT, initial=self.LOW)

    def close(self):
        self.cleanup()
        try:
            lgpio.gpiochip_close(self._h)
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
