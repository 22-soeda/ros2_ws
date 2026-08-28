# 敵機の検出結果。opponent_detector → behavior。
#
# 型の出どころ: 技術ノート(3)『自律機の動作計画指針』§5.1 の表。
# ros-architecture §4 では /opponent は geometry_msgs/PointStamped だが、
# 転倒判定に上端高さ top_height が要るのでこの型へ昇格させたもの。
#
# 座標系: 機体座標 Σ_B（x 前・y 左・z 上）。ただし z の原点はリング面で、
# 機体原点ではない。カメラは機体に固定なので、検出器は
# 「リング面の法線から作った鉛直 u」で高さを測り、カメラの取り付けオフセット
# （opponent_detector の cam_offset_x/y）を引いて機体原点へ移して出す。
# 傾いても方位の意味が変わらないのがこの座標系を採る理由（技術ノート(4) §2.2）。

std_msgs/Header header

# --- 検出器の状態 ---------------------------------------------------------
# 技術ノート(4) §9.2。「相手なし」を 1 つに潰さないための区別で、行動層は
# NO_OPPONENT では通常の探索に入り、ATTITUDE_STALE / RING_LOST では
# 知覚が壊れている前提で別の扱い（旋回を遅くする等）をする。
uint8 STATUS_OK=0              # リング面も姿勢も正常で、相手が見えている
uint8 STATUS_NO_OPPONENT=1     # 正常だが条件を通るクラスタがない（＝視野に相手がいない）
uint8 STATUS_ATTITUDE_STALE=2  # リング面は取れたが面あてはめの門を通らない。姿勢はジャイロ任せで劣化中
uint8 STATUS_RING_LOST=3       # リング面自体が取れない。転倒直後や場外を向いているとき
uint8 status

# status == STATUS_OK と同値。行動層が最初に見る 1 ビットを型の約束として残してある。
bool valid

# 追尾が外挿に落ちている（観測がゲートを外れた／クラスタが消えた）。
# valid は立ったままなので、鮮度の目安としてこちらを見る。
bool extrapolated

# --- 相手 -----------------------------------------------------------------
geometry_msgs/Point position    # [m] 重心。z はリング面からの高さ
geometry_msgs/Vector3 velocity  # [m/s] リング面内の速度。α-β 追尾の副産物（技術ノート(4) §2.1）

float32 range                   # [m]   水平距離 ρ = hypot(x, y)
float32 bearing                 # [rad] 方位 β = atan2(y, x)。左が正
float32 top_height              # [m]   クラスタ上端のリング面からの高さ z_top。転倒判定に使う
float32 width                   # [m]   水平方向の広がり max(w, d)
