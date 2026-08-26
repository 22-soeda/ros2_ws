# -*- coding: utf-8 -*-
"""RP1 のハードウェア PWM を /sys/class/pwm 経由で叩く薄いラッパ。ROS 非依存。

前提 (2026-08-27 実機で確定):
  * /boot/firmware/config.txt に `dtoverlay=ui-pwm` (自前 overlay) がある
  * その overlay が GPIO12/13/18/19 を pinctrl-rp1 の function "pwm0" に割り当て、
    /axi/pcie@120000/rp1/pwm@98000 (= 1f00098000.pwm) を status="okay" にしている
  * systemd の ui-pwm-setup.service が起動時に ch0-3 を export し、
    root:gpio / g+rw にしている (ROS ノードを root で走らせないため)

チャンネル対応 (RP1 の alt 割り当て。実機確認は led_test.py --identify で行う):
    ch0 = GPIO12 = LED Red
    ch1 = GPIO13 = LED Green
    ch2 = GPIO18 = LED Blue
    ch3 = GPIO19 = Buzzer

pwmchipN の N はカーネル版で変わるのでハードコードしない。of_node のアドレスで引く。
"""

import os
from pathlib import Path
import time

#: RP1 PWM0 の物理アドレス。pwm@9c000 (=1f0009c000) の方は cooling_fan が使う別ブロック。
RP1_PWM0_ADDR = '1f00098000.pwm'

CH_RED = 0
CH_GREEN = 1
CH_BLUE = 2
CH_BUZZER = 3

GPIO_OF_CH = {CH_RED: 12, CH_GREEN: 13, CH_BLUE: 18, CH_BUZZER: 19}


class PwmNotAvailable(RuntimeError):
    pass


def find_chip(addr=RP1_PWM0_ADDR):
    """RP1 PWM0 の pwmchip ディレクトリを返す。見つからなければ理由付きで例外。"""
    seen = []
    for c in sorted(Path('/sys/class/pwm').glob('pwmchip*')):
        dev = c / 'device'
        if not dev.exists():
            continue
        real = os.path.realpath(dev)
        seen.append(f'{c.name} -> {real}')
        if real.endswith(addr):
            return Path(os.path.realpath(c))
    raise PwmNotAvailable(
        f'RP1 PWM0 ({addr}) が見つからない。\n'
        f'  見えている pwmchip: {seen or "(なし)"}\n'
        f'  確認: grep ui-pwm /boot/firmware/config.txt / 再起動したか / '
        f'systemctl status ui-pwm-setup.service')


class PwmChannel:
    """1 チャンネル分。周波数(Hz)とデューティ(%)で扱う。"""

    def __init__(self, chip, channel):
        self.chip = Path(chip)
        self.channel = int(channel)
        self.path = self.chip / f'pwm{self.channel}'
        if not self.path.exists():
            # systemd が export 済みのはずだが、手動起動などのために自力でも試す
            try:
                (self.chip / 'export').write_text(str(self.channel))
                time.sleep(0.05)
            except PermissionError as e:
                raise PwmNotAvailable(
                    f'pwm{self.channel} が未 export で、export も書けない: {e}\n'
                    f'  sudo systemctl start ui-pwm-setup.service を試すこと') from e
        if not self.path.exists():
            raise PwmNotAvailable(f'{self.path} が現れない')
        self._freq = None

    # --- 低レベル ---------------------------------------------------------
    def _write(self, attr, value):
        try:
            (self.path / attr).write_text(str(int(value)))
        except PermissionError as e:
            raise PwmNotAvailable(
                f'{self.path/attr} に書けない: {e}\n'
                f'  ui-pwm-setup.service が chown root:gpio していないか、'
                f'ユーザーが gpio グループに入っていない') from e

    def _read(self, attr):
        return int((self.path / attr).read_text().strip())

    # --- 高レベル ---------------------------------------------------------
    @property
    def gpio(self):
        return GPIO_OF_CH.get(self.channel, '?')

    def set_frequency(self, hz):
        """周波数を変える。duty > 新 period だとカーネルに蹴られるので先に 0 に落とす。"""
        period_ns = int(round(1e9 / float(hz)))
        self._write('duty_cycle', 0)
        self._write('period', period_ns)
        self._freq = float(hz)
        return period_ns

    def set_duty(self, percent):
        """デューティ 0-100%。GPIO High = 点灯/駆動 の正論理 (§0.2)。反転させないこと。"""
        percent = max(0.0, min(100.0, float(percent)))
        period = self._read('period')
        self._write('duty_cycle', int(period * percent / 100.0))
        return percent

    def enable(self):
        self._write('enable', 1)

    def disable(self):
        self._write('enable', 0)

    def off(self):
        """安全側に倒す。デューティ 0 → 無効化。"""
        try:
            self._write('duty_cycle', 0)
            self._write('enable', 0)
        except Exception:
            pass

    def state(self):
        return (f'pwm{self.channel}(GPIO{self.gpio}) period={self._read("period")}ns '
                f'duty={self._read("duty_cycle")}ns enable={self._read("enable")}')


def open_channels(channels, freq_hz):
    """複数チャンネルを開いて周波数を揃え、デューティ 0・無効の状態で返す。"""
    chip = find_chip()
    out = {}
    for ch in channels:
        c = PwmChannel(chip, ch)
        c.set_frequency(freq_hz)
        c.set_duty(0)
        c.disable()
        out[ch] = c
    return chip, out


def all_off(channels):
    for c in channels.values() if isinstance(channels, dict) else channels:
        c.off()
