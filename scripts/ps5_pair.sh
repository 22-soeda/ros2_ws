#!/usr/bin/env bash
# PS5 (DualSense) コントローラを Bluetooth でペアリング・接続する。
#
#   使い方:  ./scripts/ps5_pair.sh              … 探して繋ぐ
#            ./scripts/ps5_pair.sh XX:XX:...    … MAC を直接指定
#            ./scripts/ps5_pair.sh --forget     … 登録済みの DualSense を削除
#
# コントローラ側の操作 (これをやらないと見つからない):
#   本体の PS ボタンと Create ボタン (十字キーの上、左上の小さいボタン) を
#   同時に 5 秒ほど押し続ける。タッチパッド脇のライトバーが速く点滅したら
#   ペアリングモード。
#
# 一度 trust すれば以後は PS ボタンを押すだけで自動接続する。
set -uo pipefail

SCAN_SECONDS=${SCAN_SECONDS:-20}
NAME_RE='DualSense|Wireless Controller'

need() { command -v "$1" >/dev/null || { echo "!! $1 が無い: sudo apt install $2"; exit 1; }; }
need bluetoothctl bluez

if ! systemctl is-active --quiet bluetooth; then
  echo "-- bluetooth サービスを起動"
  sudo systemctl enable --now bluetooth || exit 1
  sleep 2
fi
bluetoothctl power on >/dev/null

known_mac() {
  bluetoothctl devices 2>/dev/null | grep -Ei "$NAME_RE" | awk '{print $2}' | head -1
}

if [[ "${1:-}" == "--forget" ]]; then
  mac=$(known_mac)
  [[ -z "$mac" ]] && { echo "登録済みの DualSense は無い"; exit 0; }
  echo "-- $mac を削除"
  bluetoothctl remove "$mac"
  exit $?
fi

MAC="${1:-}"

if [[ -z "$MAC" ]]; then
  MAC=$(known_mac)
  if [[ -n "$MAC" ]]; then
    echo "-- 登録済みのコントローラ $MAC を使う"
  else
    echo "-- ${SCAN_SECONDS}秒スキャンする。今のうちにコントローラの PS + Create を"
    echo "   5 秒ほど同時押しして、ライトバーを速い点滅にすること"
    bluetoothctl --timeout "$SCAN_SECONDS" scan on >/dev/null 2>&1
    MAC=$(known_mac)
  fi
fi

if [[ -z "$MAC" ]]; then
  echo "!! DualSense が見つからない。ペアリングモード (PS + Create 長押し、ライトバーが"
  echo "   速い点滅) になっているか確認して、もう一度実行する"
  exit 1
fi

echo "-- 対象: $MAC"
bluetoothctl --timeout 5 agent on          >/dev/null 2>&1
bluetoothctl --timeout 5 default-agent     >/dev/null 2>&1

# 既にペア済みなら pair は失敗するので、失敗しても止めない (trust/connect は打つ)。
bluetoothctl --timeout 30 pair    "$MAC" 2>&1 | tail -2
bluetoothctl --timeout 10 trust   "$MAC" 2>&1 | tail -1
bluetoothctl --timeout 20 connect "$MAC" 2>&1 | tail -2

echo "-- 接続の確認"
for _ in $(seq 1 10); do
  if ls /dev/input/js* >/dev/null 2>&1; then break; fi
  sleep 1
done

if ls /dev/input/js* >/dev/null 2>&1; then
  ls -l /dev/input/js*
  # hid-playstation が掴んでいれば DualSense として正しく認識されている
  grep -l . /sys/class/input/js*/device/name 2>/dev/null | while read -r f; do
    echo "   $(cat "$f")"
  done
  echo "OK. 続けて:  ros2 launch roboone_teleop teleop.launch.py"
else
  echo "!! /dev/input/js* が出てこない。bluetoothctl info $MAC で Connected を確認する"
  exit 1
fi
