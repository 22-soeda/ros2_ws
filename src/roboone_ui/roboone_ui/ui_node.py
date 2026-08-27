# -*- coding: utf-8 -*-
"""ui ノード — OLED / RGB LED / 圧電サウンダーで機体状態を提示する。

設計の要点 (ブリーフ §1「ノードの構造」。設計判断済みなので従うこと):

  * タイマーは 2 本だけ。スレッドは足さない。
      OLED 再描画  10Hz … _pending != _shown のときだけ描く
      LED/ブザー   50Hz … 点滅・鳴動パターンのステートマシンを進める
  * SSD1331 の全画面書き換えは 96x64x16bit = 12,288 バイトで、実測 15.7ms かかる
    (Gate 1)。これを subscriber コールバックの中でやると publish のたびに SPI を
    占有するので、「最新値を覚える」と「実際に描く」を分ける。
    タプル比較でデバウンスが無料で付き、同じ内容が高頻度で流れても SPI は 1 回だけ。
  * シングルスレッド executor のまま。OLED 描画が LED タイマーを十数msブロックするが、
    点滅では見えず、ビープでは聞こえない。マルチスレッド化やロック導入で複雑にしない。
  * 3 デバイスは独立に初期化する。表示はロボットの本質機能ではないので、
    OLED が挿さっていないだけで launch 全体が起動しなくなるのは困る。
  * 終了時に 3 つとも必ず止める。特にブザーの停止漏れは実害が大きい。
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from roboone_interfaces.msg import LedColor, OledText
from std_msgs.msg import String

from .buzzer import Buzzer
from .oled_ssd1331 import OledDisplay
from .rgb_led import RgbLed

# --- タイマー周期 -----------------------------------------------------------
OLED_HZ = 10.0
TICK_HZ = 50.0
TICK_MS = 1000.0 / TICK_HZ          # 20ms

# --- プリセット -------------------------------------------------------------
# ★中身は仮。名前と内容は依頼者と決める (ブリーフ §4 の未決事項)。
#   LED: (r, g, b, on_ms, off_ms) の列を繰り返す。off_ms=0 で点灯しっぱなし。
# ★"off"/"on"/"yes"/"no" は使わないこと。YAML 1.1 ではこれらが真偽値として解釈され、
#   `ros2 topic pub ... "{data: off}"` が data="False" になって黙って届かない (実測)。
LED_PATTERNS = {
    'dark':  [(0, 0, 0, 1000, 0)],
    'ready': [(0, 255, 0, 1000, 0)],
    'warn':  [(255, 180, 0, 200, 200)],
    'estop': [(255, 0, 0, 100, 100)],
    # teleop の状態表示用に追加 (roboone_teleop の README「状態表示」)。
    # 自律動作は「ロボットが自分で動く」= 人が近寄ってはいけない状態なので、
    # 手動 (ready の緑点灯) と一目で区別が付くよう青の点滅にしてある。
    'auto':  [(0, 80, 255, 400, 400)],
    'link':  [(0, 200, 255, 1000, 0)],
}

#   ブザー: (on_ms, off_ms) の列。周波数は 4kHz 固定。1 回だけ鳴って止まる。
BUZZER_PATTERNS = {
    'beep':  [(80, 0)],
    'ack':   [(60, 60), (60, 0)],
    'error': [(400, 100), (400, 0)],
}

#: latched QoS。ui ノードを後から起動しても最後の指令が届き、無表示のままにならない。
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class UiNode(Node):

    def __init__(self):
        super().__init__('ui')

        # --- デバイス初期化 (1 つの失敗が他を巻き込まないようにする) ---------
        self._oled = None
        self._led = None
        self._buzzer = None
        for name, ctor in (('oled', OledDisplay), ('led', RgbLed), ('buzzer', Buzzer)):
            try:
                setattr(self, f'_{name}', ctor())
                self.get_logger().info(f'{name} 初期化 OK')
            except Exception as e:
                self.get_logger().warn(f'{name} 初期化失敗、無効化して続行: {e}')
                setattr(self, f'_{name}', None)

        if self._oled is not None:
            self.get_logger().info(
                f'OLED {self._oled.size[0]}x{self._oled.size[1]}, '
                f'1行 {self._oled.chars_per_line} 文字 (超過分はクリップ)')

        # --- 絵の前変換 (表示要求時に変換処理を走らせない) -------------------
        self._images = self._preload_images()

        # --- 表示状態 -------------------------------------------------------
        self._pending = None      # 最新の表示要求
        self._shown = None        # 実際に画面に出ているもの

        # --- パターンのステートマシン ---------------------------------------
        self._led_steps = None
        self._led_i = 0
        self._led_phase = 'on'
        self._led_left = 0.0

        self._bz_steps = None
        self._bz_i = 0
        self._bz_phase = 'on'
        self._bz_left = 0.0

        # --- 購読 (3 デバイスとも「直接指定」と「プリセット名」の 2 系統) ----
        self.create_subscription(OledText, '/ui/oled/text', self.on_oled_text, LATCHED)
        self.create_subscription(String, '/ui/oled/image', self.on_oled_image, LATCHED)
        self.create_subscription(LedColor, '/ui/led', self.on_led, LATCHED)
        self.create_subscription(String, '/ui/led/pattern', self.on_led_pattern, LATCHED)
        self.create_subscription(String, '/ui/buzzer', self.on_buzzer, LATCHED)

        # --- タイマー 2 本 ---------------------------------------------------
        self.create_timer(1.0 / OLED_HZ, self.redraw)
        self.create_timer(1.0 / TICK_HZ, self.tick)

        self.get_logger().info('ui ノード開始')

    # ======================================================================
    # 起動時処理
    # ======================================================================
    def _preload_images(self):
        """images/*.png を全部読んで表示形式へ前変換し、メモリに置く。"""
        images = {}
        size = self._oled.size if self._oled is not None else (96, 64)
        mode = self._oled.mode if self._oled is not None else 'RGB'
        try:
            share = get_package_share_directory('roboone_ui')
        except Exception as e:
            self.get_logger().warn(f'パッケージ share ディレクトリが引けない: {e}')
            return images
        img_dir = os.path.join(share, 'images')
        if not os.path.isdir(img_dir):
            self.get_logger().warn(f'画像ディレクトリが無い: {img_dir}')
            return images
        for fn in sorted(os.listdir(img_dir)):
            stem, ext = os.path.splitext(fn)
            if ext.lower() != '.png':
                continue
            try:
                img = Image.open(os.path.join(img_dir, fn))
                if img.size != size:
                    img = img.resize(size)
                images[stem] = img.convert(mode)
            except Exception as e:
                self.get_logger().warn(f'画像 {fn} を読めない: {e}')
        self.get_logger().info(f'画像 {len(images)} 枚を前変換: {sorted(images)}')
        return images

    # ======================================================================
    # subscriber コールバック — 最新値を覚えるだけ。ここでは描かない。
    # ======================================================================
    def on_oled_text(self, msg):
        self._pending = ('text', msg.line1, msg.line2, (msg.r, msg.g, msg.b))

    def on_oled_image(self, msg):
        name = msg.data
        if name not in self._images:
            # 未知のプリセット名は警告 1 行で現状維持。例外を投げない (ブリーフ §1)。
            self.get_logger().warn(
                f'未知の画像名 "{name}"、現状維持。ある画像: {sorted(self._images)}')
            return
        self._pending = ('image', name)

    def on_led(self, msg):
        # 直接指定はパターンを解除する
        self._led_steps = None
        if self._led is not None:
            self._led.set_rgb(msg.r, msg.g, msg.b)

    def on_led_pattern(self, msg):
        name = msg.data
        steps = LED_PATTERNS.get(name)
        if steps is None:
            self.get_logger().warn(
                f'未知の LED パターン "{name}"、現状維持。ある名前: {sorted(LED_PATTERNS)}')
            return
        self._led_steps = steps
        self._led_enter(0, 'on')

    def on_buzzer(self, msg):
        name = msg.data
        steps = BUZZER_PATTERNS.get(name)
        if steps is None:
            self.get_logger().warn(
                f'未知のブザーパターン "{name}"、現状維持。ある名前: {sorted(BUZZER_PATTERNS)}')
            return
        self._bz_steps = steps
        self._bz_enter(0, 'on')

    # ======================================================================
    # タイマー 1: OLED 再描画 (10Hz)
    # ======================================================================
    def redraw(self):
        if self._oled is None or self._pending == self._shown:
            return                                  # 変化なしなら描かない
        try:
            kind = self._pending[0]
            if kind == 'text':
                _, line1, line2, color = self._pending
                self._oled.show_text(line1, line2, color)
            elif kind == 'image':
                self._oled.show_image(self._images[self._pending[1]])
        except Exception as e:
            self.get_logger().warn(f'OLED 描画失敗: {e}')
        # 成否によらず _shown を進める。失敗を 10Hz で再試行してログを埋めないため。
        self._shown = self._pending

    # ======================================================================
    # タイマー 2: LED / ブザーのパターン (50Hz)
    # ======================================================================
    def tick(self):
        self._tick_led()
        self._tick_buzzer()

    # --- LED: 繰り返し ----------------------------------------------------
    def _led_enter(self, i, phase):
        r, g, b, on_ms, off_ms = self._led_steps[i]
        if self._led is not None:
            self._led.set_rgb(*((r, g, b) if phase == 'on' else (0, 0, 0)))
        self._led_i = i
        self._led_phase = phase
        self._led_left = float(on_ms if phase == 'on' else off_ms)

    def _tick_led(self):
        if self._led_steps is None:
            return
        self._led_left -= TICK_MS
        if self._led_left > 0:
            return
        steps = self._led_steps
        i = self._led_i
        if self._led_phase == 'on':
            off_ms = steps[i][4]
            if off_ms > 0:
                self._led_enter(i, 'off')
            elif len(steps) == 1:
                self._led_left = float(steps[0][3])   # 単一ステップ・消灯なし = 点灯継続
            else:
                self._led_enter((i + 1) % len(steps), 'on')
        else:
            self._led_enter((i + 1) % len(steps), 'on')

    # --- ブザー: 1 回だけ鳴って止まる -------------------------------------
    def _bz_enter(self, i, phase):
        on_ms, off_ms = self._bz_steps[i]
        if self._buzzer is not None:
            self._buzzer.on() if phase == 'on' else self._buzzer.off()
        self._bz_i = i
        self._bz_phase = phase
        self._bz_left = float(on_ms if phase == 'on' else off_ms)

    def _bz_stop(self):
        self._bz_steps = None
        if self._buzzer is not None:
            self._buzzer.off()

    def _tick_buzzer(self):
        if self._bz_steps is None:
            return
        self._bz_left -= TICK_MS
        if self._bz_left > 0:
            return
        steps = self._bz_steps
        i = self._bz_i
        if self._bz_phase == 'on':
            off_ms = steps[i][1]
            if off_ms > 0:
                self._bz_enter(i, 'off')
            elif i + 1 < len(steps):
                self._bz_enter(i + 1, 'on')
            else:
                self._bz_stop()
        else:
            if i + 1 < len(steps):
                self._bz_enter(i + 1, 'on')
            else:
                self._bz_stop()

    # ======================================================================
    # 終了処理 — 3 つとも必ず止める
    # ======================================================================
    def _shutdown_log(self, msg):
        """終了時のログ。SIGINT では rclpy が先に context を畳むので rosout へ出せない。

        その状態で get_logger() を使うと 1 行ごとに
        "Failed to publish log message to rosout" が混ざるため、stderr に落とす。
        """
        if rclpy.ok():
            self.get_logger().info(msg)
        else:
            print(f'[ui] {msg}', file=sys.stderr)

    def destroy_node(self):
        for name in ('_buzzer', '_led', '_oled'):     # ブザーを最優先で止める
            dev = getattr(self, name, None)
            if dev is None:
                continue
            try:
                dev.close()
                self._shutdown_log(f'{name[1:]} 停止')
            except Exception as e:
                self._shutdown_log(f'{name[1:]} 停止失敗: {e}')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UiNode()
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
