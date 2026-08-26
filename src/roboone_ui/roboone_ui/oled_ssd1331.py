# -*- coding: utf-8 -*-
"""OLED (秋月 QT095B / SSD1331 / 96x64) ドライバ。ROS 非依存。

Gate 1 で実機検証済み (2026-08-27):
  * cs-mode=hold-low, SPI 8MHz で初期化成功
  * 全画面書き換え avg 15.71ms (min 14.63 / max 16.77)

配線 (回路リテイクで確定。変更禁止):
  VDD=+3.3V (絶対最大定格 4V。5V 系に繋がない)  VSS=GND
  SCLK=GPIO11(SPI0)  SDIN=GPIO10(SPI0 MOSI)
  RES=GPIO25  D/C=GPIO24  CS=GPIO23 ← ハードウェア CE0(GPIO8) ではない

CS はソフト制御。SPI0 バス上に他デバイスが無いので GPIO23 を Low に落としたまま
放置する (ブリーフ §0.1)。spidev が使わない CE0(GPIO8) をトグルするが未接続で無害。
"""

from luma.core.interface.serial import spi
from luma.core.legacy import text as legacy_text
from luma.core.legacy.font import CP437_FONT
from luma.oled.device import ssd1331
from PIL import Image, ImageDraw

from .lgpio_shim import LgpioBackend

# --- 配線 (§0.1) -----------------------------------------------------------
GPIO_CS = 23
GPIO_DC = 24
GPIO_RST = 25
SPI_PORT = 0
SPI_DEVICE = 0
SPI_SPEED_HZ = 8000000

# --- フォント (2026-08-27 実測) --------------------------------------------
# CP437_FONT は 8x8。96px / 8px = 1 行ちょうど 12 文字。
FONT = CP437_FONT
FONT_W = 8
FONT_H = 8

#: 2 行の描画位置。画面 64px の中で上下に振り分ける。
LINE1_XY = (0, 20)
LINE2_XY = (0, 36)


class OledDisplay:
    """SSD1331 の 2 行テキスト + 画像表示。

    表示の仕様 (ブリーフ §1。拡張しないこと):
      * 2 行固定・フォントサイズ固定。3 行目/スクロール/可変サイズは作らない
      * 色はメッセージ単位で 1 色。行ごとの色指定は作らない
      * 画面幅からのはみ出しはクリップして無視する。エラーにも警告にもしない
    """

    def __init__(self, cs=GPIO_CS, dc=GPIO_DC, rst=GPIO_RST,
                 port=SPI_PORT, device=SPI_DEVICE, speed_hz=SPI_SPEED_HZ,
                 reset_hold_time=0.100, reset_release_time=0.150):
        self._gpio = LgpioBackend()
        self._serial = None
        self._dev = None
        try:
            # CS を Low で保持し続ける
            self._gpio.hold_low(cs)
            self._serial = spi(gpio=self._gpio, port=port, device=device,
                               bus_speed_hz=speed_hz, gpio_DC=dc, gpio_RST=rst,
                               reset_hold_time=reset_hold_time,
                               reset_release_time=reset_release_time)
            # 既定の差分描画 (diff_to_previous) のまま使う。全画面転送より速い。
            self._dev = ssd1331(self._serial)
        except Exception:
            self.close()
            raise

    # --- 情報 -------------------------------------------------------------
    @property
    def size(self):
        return self._dev.size

    @property
    def mode(self):
        return self._dev.mode

    @property
    def chars_per_line(self):
        """1 行に入る文字数。これを超えた分は黙って切れる。"""
        return self._dev.size[0] // FONT_W

    # --- 描画 -------------------------------------------------------------
    def show_text(self, line1, line2, color):
        """2 行を 1 色で描く。color は (r, g, b) 0-255。"""
        img = Image.new(self._dev.mode, self._dev.size, (0, 0, 0))
        draw = ImageDraw.Draw(img)
        # はみ出しは PIL 側で自然にクリップされる。ここでは何もしない (仕様)。
        legacy_text(draw, LINE1_XY, self._sanitize(line1), fill=color, font=FONT)
        legacy_text(draw, LINE2_XY, self._sanitize(line2), fill=color, font=FONT)
        self._dev.display(img)

    def show_image(self, img):
        """前変換済み PIL Image を出す。変換はここではやらない (起動時に済ませる)。"""
        self._dev.display(img)

    def clear(self):
        if self._dev is not None:
            self._dev.clear()

    @staticmethod
    def _sanitize(s):
        """CP437 フォントは ASCII 前提。描けない文字は '?' に落とす。

        日本語表示は不要と確定済み (2026-08-27)。よってフォント同梱の検討も不要。
        """
        if not s:
            return ''
        return ''.join(c if 32 <= ord(c) < 127 else '?' for c in s)

    # --- 後始末 -----------------------------------------------------------
    def close(self):
        """画面を消してから資源を返す。焼き付き防止 (ブリーフ §1)。"""
        try:
            if self._dev is not None:
                self._dev.clear()
        except Exception:
            pass
        try:
            if self._serial is not None:
                self._serial.cleanup()
        except Exception:
            pass
        try:
            self._gpio.close()
        except Exception:
            pass
