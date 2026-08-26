#!/bin/sh
# RP1 PWM0 のチャンネルを起動時に export し、gpio グループから書けるようにする。
#
# 目的: ROS ノードを root で走らせないため (ブリーフ §3 の禁止事項)。
# Ubuntu の /lib/udev/rules.d/60-gpio.rules には pwm のルールが無いので、
# /sys/class/pwm は root 専用のまま。ここで補う。
#
# 安全性: export と period/duty_cycle=0 の設定のみ。enable は 0 のままなので
# 出力は出ない (LED 消灯・ブザー無音)。
set -eu

GROUP=gpio
PERIOD_NS=250000        # 4 kHz。LED は IEEE 1789-2015 で 3kHz 超が推奨、
                        # ブザーは PKM13EPYH4000-A0 の共振周波数が 4.0kHz。
CHANNELS="0 1 2 3"      # GPIO12=ch0(R) 13=ch1(G) 18=ch2(B) 19=ch3(Buzzer)

# RP1 PWM0 (1f00098000.pwm) の pwmchip を探す。番号はカーネル版で変わるのでハードコードしない。
CHIP=""
for c in /sys/class/pwm/pwmchip*; do
    [ -e "$c/device" ] || continue
    case "$(readlink -f "$c/device")" in
        *1f00098000.pwm) CHIP="$(readlink -f "$c")" ;;
    esac
done

if [ -z "$CHIP" ]; then
    echo "ui-pwm-setup: RP1 PWM0 (1f00098000.pwm) が見つからない。" >&2
    echo "  config.txt の dtoverlay=ui-pwm と再起動を確認すること。" >&2
    for c in /sys/class/pwm/pwmchip*; do
        echo "  見えているもの: $c -> $(readlink -f "$c/device" 2>/dev/null)" >&2
    done
    exit 1
fi
echo "ui-pwm-setup: chip=$CHIP npwm=$(cat "$CHIP/npwm")"

for ch in $CHANNELS; do
    [ -d "$CHIP/pwm$ch" ] || echo "$ch" > "$CHIP/export" || {
        echo "ui-pwm-setup: export $ch 失敗" >&2; continue; }
    # period を先に書く (duty <= period の制約があるため)
    echo 0          > "$CHIP/pwm$ch/enable"      2>/dev/null || true
    echo 0          > "$CHIP/pwm$ch/duty_cycle"  2>/dev/null || true
    echo "$PERIOD_NS" > "$CHIP/pwm$ch/period"
    echo 0          > "$CHIP/pwm$ch/duty_cycle"
    echo 0          > "$CHIP/pwm$ch/enable"
    echo "ui-pwm-setup: pwm$ch period=$(cat "$CHIP/pwm$ch/period") duty=$(cat "$CHIP/pwm$ch/duty_cycle") enable=$(cat "$CHIP/pwm$ch/enable")"
done

# 権限付与。sysfs は chown/chmod が効く。
chown -R "root:$GROUP" "$CHIP" 2>/dev/null || true
chmod -R g+rwX "$CHIP"         2>/dev/null || true
# unexport/export 自体も gpio グループから叩けるようにしておく
chgrp "$GROUP" "$CHIP/export" "$CHIP/unexport" 2>/dev/null || true
chmod g+w     "$CHIP/export" "$CHIP/unexport" 2>/dev/null || true

echo "ui-pwm-setup: done"
