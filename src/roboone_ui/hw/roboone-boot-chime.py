#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""起動確認用のブザーメロディ。systemd から boot 時に 1 回だけ鳴らす。

★このスクリプトは意図的に自己完結させてある (ROS にも roboone_ui にも依存しない)
    目的が「ラズパイがちゃんと起動したか」の確認なので、colcon ワークスペースが
    壊れていても、install/ が無くても鳴らないと意味がない。標準ライブラリのみ、
    /sys/class/pwm を直接叩く。rp1_pwm.py と処理が重複するが、そこは承知の上。

★音階が 3.5k-4.7kHz に固まっているのも、1 音が 80ms 以上あるのも音量のため
    BZ1 = 村田 PKM13EPYH4000-A0 は共振 4.0kHz の外部駆動型で、共振から離れるほど
    音圧が落ちる。加えて聴覚は 200ms 以下の音を短いほど小さく感じる (時間積分)。
    最初は 3.1k-5.3k / 1 音 45ms で書いたが実機で「小さくて聞こえにくい」だったので、
    帯域を共振の ±17% に詰め、音長を倍以上にした。音色がチャイム/オルゴール寄りの
    高音になり、音程の幅が狭いのはこの制約による。仕様として受け入れる。
    個体の共振が 4.0kHz からずれている可能性は --sweep で確認できる。

前提: dtoverlay=ui-pwm と ui-pwm-setup.service (ch3 の export と gpio 権限付与)。
      どちらも roboone_ui/README.md の「ハードウェアの有効化」で入れる。
配線:  GPIO19 = RP1 PWM0 ch3 → Q2 (2SC2712) ローサイド。GPIO High = 鳴動の正論理。
"""

import argparse
import os
from pathlib import Path
import sys
import time

RP1_PWM0_ADDR = '1f00098000.pwm'
CH_BUZZER = 3

#: ui-pwm-setup.sh が置いていく既定値。終わったらここへ戻す。
IDLE_PERIOD_NS = 250000         # 4kHz
DEFAULT_DUTY = 50               # 矩形波は 50% が基本波成分が最大 = 一番大きい

# 共振 4.0kHz の ±17% に収まる音だけを使う。括弧内は共振からのずれ。
A7 = 3520   # -12%
B7 = 3951   # -1%。ほぼ共振で一番大きく鳴る
C8 = 4186   # +5%
D8 = 4699   # +17%
# 下 2 つは 'wide' 専用。音程の幅は出るが、聴感で明らかに小さい
G7 = 3136   # -22%
E8 = 5274   # +32%
REST = 0

#: (周波数Hz, 長さms)。周波数 0 は休符。締めの 1 音を長くすると全体が大きく聞こえる。
MELODIES = {
    # 上昇 3 音 + 共振付近で締める「ポロロン♪」。速い分やや聞き逃しやすい
    'pororon': [(A7, 80), (C8, 80), (D8, 80), (REST, 20), (C8, 340)],
    # ★採用 (2026-08-28 実機の聴感で決定)。1 音ずつ長めに立てるので一番聞き取りやすい
    'chime':   [(A7, 140), (REST, 30), (C8, 140), (REST, 30), (D8, 140), (REST, 30),
                (C8, 400)],
    # 跳ねる。ピッ・ピッ・ポーン
    'skip':    [(C8, 90), (REST, 45), (C8, 90), (REST, 45), (D8, 320)],
    # 下降。停止側に使うなら
    'down':    [(D8, 90), (C8, 90), (A7, 90), (REST, 20), (B7, 320)],
    # 音域を広く取った版。メロディらしさは上だが両端が小さい (上の★を参照)
    'wide':    [(G7, 90), (A7, 90), (C8, 90), (D8, 90), (E8, 90), (REST, 20), (C8, 340)],
}
DEFAULT_MELODY = 'chime'

#: --sweep で鳴らす帯。この個体の共振が本当に 4.0kHz かを耳で確かめるため。
SWEEP_HZ = (3400, 3600, 3800, 4000, 4200, 4400, 4600)


class Pwm:
    """ch3 を鳴らす最小限のラッパ。with で必ず消音して戻す。"""

    def __init__(self, channel=CH_BUZZER):
        self.chip = self._find_chip()
        self.path = self.chip / f'pwm{channel}'
        if not self.path.exists():
            # 通常は ui-pwm-setup.service が export 済み。手動起動などのために自力でも試す
            (self.chip / 'export').write_text(str(channel))
            time.sleep(0.05)
        if not self.path.exists():
            raise RuntimeError(f'{self.path} が現れない')

    @staticmethod
    def _find_chip():
        seen = []
        for c in sorted(Path('/sys/class/pwm').glob('pwmchip*')):
            dev = c / 'device'
            if not dev.exists():
                continue
            real = os.path.realpath(dev)
            seen.append(f'{c.name} -> {real}')
            if real.endswith(RP1_PWM0_ADDR):
                return Path(os.path.realpath(c))
        raise RuntimeError(
            f'RP1 PWM0 ({RP1_PWM0_ADDR}) が見つからない。'
            f'config.txt の dtoverlay=ui-pwm と再起動を確認すること。'
            f' 見えている pwmchip: {seen or "(なし)"}')

    def _write(self, attr, value):
        (self.path / attr).write_text(str(int(value)))

    def tone(self, hz, ms, duty=DEFAULT_DUTY):
        if hz <= 0:                       # 休符。enable は保ったまま出力だけ落とす
            self._write('duty_cycle', 0)
        else:
            period = int(round(1e9 / float(hz)))
            # duty > period はカーネルに蹴られるので、先に duty を 0 に落としてから period
            self._write('duty_cycle', 0)
            self._write('period', period)
            self._write('duty_cycle', period * duty // 100)
            self._write('enable', 1)
        time.sleep(ms / 1000.0)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        """鳴りっぱなしを絶対に残さない。period も ui-pwm-setup の既定へ戻す。"""
        for attr, val in (('duty_cycle', 0), ('enable', 0), ('period', IDLE_PERIOD_NS)):
            try:
                self._write(attr, val)
            except Exception:
                pass
        return False


def main(argv=None):
    ap = argparse.ArgumentParser(description='起動確認ブザー (RP1 PWM0 ch3 / GPIO19)')
    ap.add_argument('-m', '--melody', default=DEFAULT_MELODY, choices=sorted(MELODIES),
                    help=f'鳴らすメロディ (既定: {DEFAULT_MELODY})')
    ap.add_argument('-d', '--duty', type=int, default=DEFAULT_DUTY,
                    help='デューティ%%。下げると小さくなる (既定: %(default)s)')
    ap.add_argument('-r', '--repeat', type=int, default=1, help='繰り返し回数')
    ap.add_argument('--delay', type=float, default=0.0,
                    help='鳴らす前の待ち秒。boot 直後に他の音と重ねたくないとき')
    ap.add_argument('--sweep', action='store_true',
                    help='共振探し。3.4k-4.6kHz を等しい長さで鳴らす (一番大きい所が共振)')
    args = ap.parse_args(argv)

    if args.delay > 0:
        time.sleep(args.delay)
    try:
        with Pwm() as pwm:
            if args.sweep:
                for hz in SWEEP_HZ:
                    print(f'{hz} Hz', flush=True)
                    pwm.tone(hz, 500, args.duty)
                    pwm.tone(REST, 400)
                return 0
            for _ in range(max(1, args.repeat)):
                for hz, ms in MELODIES[args.melody]:
                    pwm.tone(hz, ms, args.duty)
    except PermissionError as e:
        # 権限は「gpio グループに入っていない」か「setup.service が走っていない」の 2 択
        print(f'boot-chime: PWM に書けない: {e}\n'
              f'  systemctl status ui-pwm-setup.service / id -nG を確認すること', file=sys.stderr)
        return 1
    except Exception as e:
        print(f'boot-chime: 鳴らせない: {e}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
