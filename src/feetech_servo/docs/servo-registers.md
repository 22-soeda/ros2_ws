# 実機サーボのレジスタ設定（2026-08-26 実測）

18軸すべての `addr 0..55` を読み出して比較した結果（この時点では bus1 ID7 が配線の接触不良で
応答していなかった。2026-08-27 に復旧後に照合し、**自分のID(addr5)と角度リミット以外は
他の 4618 軸と完全一致**を確認済み）。**バス0とバス1は完全に同じ値**で、
差があるのは**型番（4618 / 5130）の間だけ**（工場出荷値の違い）。★印がその差。

※ 角度リミット（addr 9 / 11）はこの実測のあと、2026-08-27 に軸ごとの可動域を書き込んだ。
現物は [config/servo_limits.yaml](../config/servo_limits.yaml)、書き込みは `feetech_set_limits`。

トルク・目標位置・速度・加速度（`addr 40..47`）は実行時に書き換わる指令値なので、この表には載せない。

## 軸構成

| バス | ポート | 軸 | model 4618 | model 5130 |
|---|---|---|---|---|
| bus0 | `/dev/ttyACM0` | ID 1-6, 8-10（9軸） | ID 1,2,3,4,8,9 | ID 5,6,10 |
| bus1 | `/dev/ttyACM1` | ID 1-10（10軸） | ID 1,2,3,4,7,8,9 | ID 5,6,10 |

計19軸（4618 が13軸、5130 が6軸）。**ID7 は bus1 のみ**、bus0 は欠番。全軸 FW 3.43 / HLS 系。

## 設定値

| addr | レジスタ | 4618（12軸） | 5130（6軸） | 内容 |
|---|---|---|---|---|
| 0 | `FIRMWARE_VER` | 3.43 | 3.43 | ファームウェア版数 |
| 3 | `MODEL` | 4618 | 5130 **★** | 型番 |
| 5 | `ID` | 軸ごと | 軸ごと | サーボID |
| 6 | `BAUD_RATE` | 0 | 0 | 0 = 1 Mbps |
| 7 | `SECOND_ID` | 253 | 253 | 第2ID（253 = 未使用）。※SMS/STS の `RETURN_DELAY` ではない |
| 8 | `RESPONSE_LEVEL` | 1 | 1 | 応答レベル |
| 9 | `MIN_ANGLE_LIMIT` | 軸ごと | 軸ごと | 可動範囲の下限。2026-08-27 に16軸へ書き込み済み（[config/servo_limits.yaml](../config/servo_limits.yaml) が現物） |
| 11 | `MAX_ANGLE_LIMIT` | 軸ごと | 軸ごと | 可動範囲の上限。bus0 ID4 と bus1 ID3 のみ `0..0`（制限なし） |
| 13 | `MAX_TEMP_LIMIT` | 80 | 80 | 温度上限 degC |
| 14 | `MAX_INPUT_VOLT` | 160 | 160 | 入力電圧上限 16.0V |
| 15 | `MIN_INPUT_VOLT` | 80 | 40 **★** | 入力電圧下限 8.0V / 4.0V |
| 16 | `MAX_TORQUE` | 980 | 1000 **★** | 最大トルク（`TORQUE_LIMIT` の電源投入時の元値） |
| 18 | `PHASE` | 80 | 116 **★** | 相設定 |
| 19 | `UNLOADING_COND` | 12 | 13 **★** | 脱力する保護条件のビット |
| 20 | `LED_ALARM_COND` | 13 | 0 **★** | LED警告を出す条件のビット |
| 21 | `P_COEF` | 32 | 32 | 位置ループ P ゲイン |
| 22 | `D_COEF` | 32 | 32 | 位置ループ D ゲイン |
| 23 | `I_COEF` | 0 | 0 | 位置ループ I ゲイン。**0 = 積分なし**（負荷方向に定常誤差が残る理由） |
| 24 | `MIN_STARTUP_FORCE` | 0 | 16 **★** | 起動最小出力 |
| 26 | `CW_DEAD` | 0 | 0 | CW側 不感帯 |
| 27 | `CCW_DEAD` | 0 | 0 | CCW側 不感帯 |
| 28 | `PROTECTION_CURRENT` | 1000 | 500 **★** | 過電流保護のしきい値 |
| 30 | `ANGULAR_RESOLUTION` | 1 | 1 | 角度分解能の倍率 |
| 31 | `OFS` | 0 | 0 | 位置オフセット。**0 = サーボ側に原点校正は入っていない** |
| 33 | `MODE` | 0 | 0 | 0 = 位置制御 |
| 34 | `PROTECTIVE_TORQUE` | 30 | 50 **★** | 保護動作後の出力 |
| 35 | `PROTECTION_TIME` | 10 | 10 | 保護判定までの時間 |
| 36 | `OVERLOAD_TORQUE` | 255 | 255 | 過負荷判定トルク |
| 37 | `MODE1_P_COEF` | 100 | 60 **★** | 速度モードの P ゲイン |
| 38 | `OVERCURRENT_PROT_TIME` | 200 | 200 | 過電流保護までの時間 |
| 39 | `MODE1_I_COEF` | 200 | 20 **★** | 速度モードの I ゲイン |
| 48 | `TORQUE_LIMIT` | 980 | 1000 **★** | トルク上限（電源投入時に `MAX_TORQUE` から復帰） |
| 55 | `LOCK` | 1 | 1 | 1 = EEPROM ロック中 |

- レジスタ名は SDK の HLS 定義（`vendor/scservo/include/scservo/HLSCL.h`）を優先し、
  HLS で未定義のものは SMS/STS 定義（`SMS_STS.h`）からの推定。addr 9/11/26/27/31/33/48/55 等は HLS 定義で確認済み。
- 電源投入時の指令値の既定は 4618 が `GOAL_TORQUE=1000, GOAL_SPEED=110`、5130 が `GOAL_TORQUE=500, GOAL_SPEED=250`。
  現状のコードは全軸に `goal_torque=1000` を書く（`TORQUE_LIMIT` 内なので範囲外ではない）。

## 読み出し方

```
ros2 run feetech_servo feetech_shell
[bus0 id-]> info @5        # 主要な設定をまとめて表示
[bus0 id-]> getw 9 @5      # 生レジスタを1つだけ読む
```
