# roboone_ui

OLED (SSD1331) / フルカラー LED / 圧電サウンダーで機体状態を提示する ui ノード。

## 依存

`package.xml` に書けるのは rosdep キーがあるものだけ。残りは手で入れる。

```bash
sudo apt install python3-luma.oled python3-spidev   # rosdep キーが無い
# python3-pil / python3-lgpio は package.xml の <depend> にあるので rosdep で入る
```

pip も venv も使わない。Ubuntu 24.04 は PEP 668 で system Python が
externally-managed だが、必要なものは全部 apt にあるので問題にならない。

## ハードウェアの有効化 (初回のみ・要再起動)

RP1 のハードウェア PWM は既定で無効。`hw/` の一式で有効にする。

```bash
sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.bak
dtc -@ -I dts -O dtb -o ui-pwm.dtbo hw/ui-pwm.dts
sudo install -m 0644 ui-pwm.dtbo /boot/firmware/overlays/
echo 'dtoverlay=ui-pwm' | sudo tee -a /boot/firmware/config.txt
sudo install -m 0755 hw/ui-pwm-setup.sh /usr/local/sbin/
sudo install -m 0644 hw/ui-pwm-setup.service /etc/systemd/system/
sudo systemctl enable ui-pwm-setup.service
sudo reboot
```

Ubuntu 付属の `pwm.dtbo` / `pwm-2chan.dtbo` は**使えない**。理由は `hw/ui-pwm.dts` の
冒頭コメントに書いてある (レガシー binding で、pinctrl-rp1 の文字列 binding と形式が違う)。

確認は `bash ~/scratch/pwm_check.sh` (読み取りのみ)。

## 起動

```bash
ros2 run roboone_ui ui_node
```

## トピック

3 デバイスとも「直接指定」と「プリセット名」の 2 系統。

| デバイス | 直接指定 | プリセット |
|---|---|---|
| OLED | `/ui/oled/text` … `roboone_interfaces/OledText` | `/ui/oled/image` … `std_msgs/String` |
| RGB LED | `/ui/led` … `roboone_interfaces/LedColor` | `/ui/led/pattern` … `std_msgs/String` |
| ブザー | (無し。共振 4kHz 固定で 1 音しか出ない) | `/ui/buzzer` … `std_msgs/String` |

プリセット名は `ui_node.py` の `LED_PATTERNS` / `BUZZER_PATTERNS` にある。
未知の名前は警告 1 行を出して現状維持。例外は投げない。

### ★QoS の注意 — CLI から叩くときはフラグが要る

購読側は全部 latched (`depth=1` / `RELIABLE` / `TRANSIENT_LOCAL`)。
DURABILITY は「publisher が offer し subscriber が request する」ので、
**VOLATILE な publisher は TRANSIENT_LOCAL な subscriber とマッチしない。**
`ros2 topic pub` の既定は VOLATILE なので、そのまま叩くと
`--once` が `-w 1` (購読者待ち) で永久に待ち続け、ノード側には

```
New publisher discovered on topic '/ui/oled/text', offering incompatible QoS.
No messages will be received from it. Last incompatible policy: DURABILITY
```

が出る。CLI からは必ず付けること:

```bash
ros2 topic pub --once --qos-durability transient_local \
  /ui/oled/text roboone_interfaces/msg/OledText \
  "{line1: 'HELLO', line2: 'ROBOONE', r: 0, g: 255, b: 0}"
```

他ノード (motion / behavior) 側も publisher を TRANSIENT_LOCAL にすること。
そうして初めて「ui を後から起動しても最後の指令が届く」が成立する。

### ★プリセット名に off / on / yes / no を使わない

YAML 1.1 ではこれらが真偽値として解釈され、`"{data: off}"` が `data="False"` に
なって黙って届かない (実測)。消灯プリセットの名前は `dark` にしてある。

## 決定事項 (2026-08-27)

- **表示は ASCII のみ。日本語表示はしない。** よって 8x8 日本語ビットマップフォントの
  同梱も不要。ASCII 外の文字は `?` に落とす (`OledDisplay._sanitize`)
- **プリセットは今のものを起点に、機能追加に合わせて足していく。** `ui_node.py` の
  `LED_PATTERNS` / `BUZZER_PATTERNS` に足すだけでよく、トピックの型は変わらない。
  現状は仮置きの `dark` / `ready` / `warn` / `estop`、`beep` / `ack` / `error`
- 画像も同様に `images/*.png` を足すだけ。起動時に自動で読み込まれる
  (`setup.py` の `data_files` は `glob` なので追記不要)

## 残っていること

- **Gate 5**: `/motion/state` を購読して 状態名 → OLED文言・LED色・ブザー音 に落とす。
  motion ノードが出来てからでよい (`docs/ros-architecture.md` §5 でも ui は最後)
- **+5V レールのテスター実測**。可変 DCDC なので 5.00V とは限らず、LED 電流に効くため
  `DEFAULT_GAIN` の最終確定に要る。現在の値は目視で問題なしと確認済み
- **LED の用途が競技規定由来かどうか**。ROBO-ONE Auto に自律動作状態のインジケータ規定が
  あるなら色・輝度・視認方向に要求があるはず。未確認

## 実測値 (2026-08-27, Raspberry Pi 5 / Ubuntu 24.04 / kernel 6.8.0-1060-raspi)

| 項目 | 値 |
|---|---|
| OLED 全画面書き換え | avg 15.71ms (min 14.63 / max 16.77)、12,288 バイト |
| 1 行の文字数 | 12 文字 (CP437 8x8 / 96px)。超過分は黙って切れる |
| LED GAIN | r=0.52 g=0.30 b=1.00 (255 指示 → duty 130000/75000/250000 ns) |
| ブザー共振 | 4kHz が 3k/5k より明確に大きい (聴感) |
| ノード CPU | 1 コアの約 4-5% (50Hz + 10Hz タイマー、待機時も同程度) |
