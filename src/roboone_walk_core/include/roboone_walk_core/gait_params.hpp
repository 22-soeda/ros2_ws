// 歩行の静的設定。roboone_walk_ref/walk_core/params.py の機械移植。
//
// **Python 版 (roboone_walk_ref) が仕様の原本。** 値やコメントを変えるときは
// 必ず両方を揃え、tools/compare_walk_engines.py で数値一致を確認すること。
//
// YAML の読み込みはここでは持たない。motion ノードが ROS パラメータ
// (gait.yaml) から詰める。既定値は gait.yaml と同じにしてある。
#ifndef ROBOONE_WALK_CORE__GAIT_PARAMS_HPP_
#define ROBOONE_WALK_CORE__GAIT_PARAMS_HPP_

#include <cmath>

namespace roboone_walk_core
{

struct GaitParams
{
  // --- 力学 -----------------------------------------------------------
  double z_c = 0.261;              // [m]   重心高さ (home_pose.yaml の foot.height 261mm と揃える)
  double gravity = 9.81;           // [m/s^2]
  // 静歩行寄りに倒した値。T を伸ばすほど重心が支持足の真上に寄る一方で e^{ωT} が
  // 増える。短くする側の下限は swing_height を出せるサーボ速度 (params.py 参照)
  double t_step = 0.60;            // [s]   1 歩の周期 T (ωT=3.68, e^{ωT}=39.6)
  double foot_spacing = 0.1786;    // [m]   左右の足間隔 W = 股間隔

  // --- 遊脚 -----------------------------------------------------------
  // 20mm では実機で床を擦るだけだった (2026-08-29)。t_step / td_speed_max と連動
  double swing_height = 0.05;      // [m]   遊脚の頂点高さ h_sw
  double swing_lock_phase = 0.70;  // [-]   着地点の凍結位相 φ_lock
  double td_overdrive = 0.004;     // [m]   名目床面より下へ突き抜ける量 z_od
  // (swing_height + td_overdrive) / (0.55 T) を下回ると足が床に届かない
  double td_speed_max = 0.20;      // [m/s] 着地直前の降下速度上限
  // 遊脚が歩周期 T のうち何割を使うか。1.0 = 従来（φ=1 ちょうどで着く）。
  // 下げるとその手前で着地点に達し、残りは両足が着いたまま止まる。
  //
  // ★ZMP は enter_step() で支持足に固定され歩の間ずっと動かないので、ここを
  //   変えても DCM の伝播も b の閉形式も a_max の発散条件も**一切変わらない**。
  //   増えるのは「支持多角形が広い時間」だけで、**計画上の重心経路は同じ**。
  //   重心の動きまで静的にしたいなら ZMP を両足間で動かす必要があり、それは
  //   b の導出からやり直しになる（この項目では届かない）。
  //
  // ★狙った値がそのまま出るわけではない。降下は td_speed_max で飽和するので、
  //   実際の着地は計画より遅れる。実際の値は checkSwingLanding() が出す。
  double swing_ratio = 1.0;        // [-]   遊脚が使う歩周期の割合

  // --- 指令の整形 -----------------------------------------------------
  // 歩幅 = v·T なので T を伸ばしたら下げること。T=0.60 で (0.15,0.08) のままだと
  // 遊脚頂点の前後が ±49mm になり 50mm 高での機構到達 (前56/後37) を超える
  double v_max[2] = {0.10, 0.04};  // [m/s] (x, y) の飽和
  // a_max は文書表 2 では (0.3, 0.2) だが、純フィードフォワードでは 1 歩あたりの
  // 指令変化 ΔL = a·T² が着地点クランプで吸収できる範囲
  //   ΔLx (1 + 1/(e^{ωT}-1)) < step_clamp_x,  ΔLy (1 + 1/(e^{ωT}+1)) < step_clamp_in
  // を超えると、吸収残りが e^{ωT} 倍に増幅されて発散する (params.py 参照)。
  // T=0.60 / step_clamp_in=0.020 での上限は (0.11, 0.05)。その 6 割。
  double a_max[2] = {0.06, 0.03};  // [m/s^2] (x, y) のレート制限

  // --- 着地点クランプ (式 11) -----------------------------------------
  double step_clamp_x = 0.04;      // [m] 前後の許容ずれ
  double step_clamp_out = 0.045;   // [m] 外側の許容ずれ
  // 停止準備歩のオフセット b_stop = (W/2)/e^{ωT} より大きく取ること (params.py 参照)
  double step_clamp_in = 0.020;    // [m] 内側の許容ずれ (脚同士の干渉のため狭い)

  // --- 状態機械 -------------------------------------------------------
  double start_pushoff_max = 0.15; // [s]   押し出しの最長時間
  double k_dcm = 1.0;              // [-]   踏み出し補正のゲイン (計画では 1)
  double cmd_timeout = 0.5;        // [s]   指令途絶で停止に入る (motion ノード側)
  double loop_hz = 200.0;          // [Hz]  周期

  // --- 閾値 (実装で追加。文書に明示値がないもの) ------------------------
  double v_start_eps = 0.005;      // [m/s] これ以上で歩き始める
  double v_stop_eps = 0.010;       // [m/s] 歩の境界でこれ未満なら停止シーケンスへ
  double settle_eps = 0.002;       // [m]   |ξ - x_C| がこれ未満で静止とみなす
  double stop_outside_eps = 0.005; // [m]   ξ が支持多角形からこれ以上外れたらもう 1 歩

  double omega() const { return std::sqrt(gravity / z_c); }
  double e_wt() const { return std::exp(omega() * t_step); }
};

}  // namespace roboone_walk_core

#endif  // ROBOONE_WALK_CORE__GAIT_PARAMS_HPP_
