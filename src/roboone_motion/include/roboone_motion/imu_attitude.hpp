// IMU の姿勢推定 — RealSense 内蔵 IMU の生値から、胴体のロール・ピッチと角速度を出す。
//
// **ROS を知らない。** motion_node が /camera/imu のコールバックで update() を呼び、
// 制御ループが attitude() を写して使う。**スレッド安全ではない**ので呼び側がロックを持つ。
//
// 外部の IMU フィルタ (imu_filter_madgwick) を挟まないのは、この Pi に入っておらず、
// プロセスを 1 段増やすとそのまま減衰項の遅れになるため
// (docs/imu_biped_walking.pdf §4.3。遅れが大きいと減衰のつもりの項が励振に回る)。
//
// ===========================================================================
// 座標系
// ===========================================================================
//   {O}  IMU の光学座標系 camera_imu_optical_frame   x 右 / y 下 / z 前
//   {N}  {O} の軸を機体の名前に並べ替えただけの系    x 前 / y 左 / z 上
//          x_n = z_o,  y_n = -x_o,  z_n = -y_o
//   {B}  機体座標 Σ_B。  R_BO = Rrpy(mount_rpy) · P_NO
//
// mount_rpy はカメラの取り付けの傾き。**pitch + でカメラが下を向く。**
// この機体は 0°（胴体に対して水平に付いている。2026-09-16 に確認）。
// roboone_perception の cam_pitch_deg = 30 は「胴体をかがめたときの光軸の俯角」の
// 初期値で、取り付けの角ではない。組み付けの誤差は立位で zero() を呼んで取る（下）。
//
// ===========================================================================
// 出力の符号 — body_pitch（home_pose.yaml）と同じ取り方
// ===========================================================================
//   pitch +  胴体が前へ倒れている（Σ_B が世界に対して y 軸まわりに + 回っている）
//   roll  +  胴体が右へ倒れている（x 軸まわりに + = 左側が上がる）
//   gyro     Σ_B で表した角速度 [rad/s]。gyro[1] がピッチの速さ、gyro[0] がロールの速さ
//
// ===========================================================================
// 推定の中身（docs/imu_biped_walking.pdf §4.1）
// ===========================================================================
// ジャイロで姿勢を積分し、加速度計の重力方向へ 1/tau_c の速さで引き戻す相補フィルタ
// （Mahony の比例項だけの形）。ヨーは観測できないので持つだけで使わない。
// ロール・ピッチは R_WB の 3 行目（= 胴体から見た鉛直）だけで決まるので、
// ヨーのドリフトは出力に効かない。
//
//   * 歩行中は加速度から出す傾きが最大 47° ずれる実測がある
//     (roboone_perception/detect/attitude.py)。対策は 2 つ:
//       - **計画上の重心の加速度を比力から引いてから**重力の向きとして使う
//         （setAccelFeedforward。LIPM なら ẍ = ω²(x_C − p) で、歩行計画が知っている）。
//         横揺れ ±2.6 m/s^2（周期 2T）をそのまま入れるとロールが 3° 以上ずれる
//       - |f| が g から外れるほど加速度を信じない（accel_band）。残りの揺れは
//         tau_c の窓で平均される
//   * 角速度は減衰項の入力なので、推定を通さず生値を出す。引くのはバイアスだけで、
//     掛けるのは 1 次 LPF だけ（gyro_lpf_hz。0 で素通し）。
//   * ジャイロのバイアスは静止している間だけ少しずつ取る。静止中の実測は最大
//     0.0048 rad/s なので bias_max で頭を抑える（ゆっくり倒れていく動きを
//     バイアスとして飲み込まないため）。
//
// ===========================================================================
// 零点（zero()）
// ===========================================================================
// 呼んだときの胴体をロール・ピッチ 0 にするよう mount_rpy を取り直す
// (docs/ros2_walk_implementation.pdf §6.1 式 (23))。
//
// **呼んだときの胴体の傾きがそのまま「0」になる。** この機体は膝のしなりで立位が
// 後傾する（home_pose.yaml の注記）ので、ホーム姿勢で立たせて呼ぶと、その後傾を
// 0 と覚えて kp（傾きの比例）が直さなくなる。カメラは胴体に水平に付いているので、
// 零点は「胴体が水平だと分かっている状態（水準器を当てる等）」で、組み付けの
// ずれを取るときだけ使う。
#ifndef ROBOONE_MOTION__IMU_ATTITUDE_HPP_
#define ROBOONE_MOTION__IMU_ATTITUDE_HPP_

#include <cstdint>
#include <string>

#include "roboone_motion/body_pose.hpp"

namespace roboone_motion
{

struct ImuOptions
{
  //! カメラの取り付けの傾き (roll, pitch, yaw) [rad]。pitch + でカメラが下を向く
  double mount_rpy[3]{0.0, 0.0, 0.0};
  double tau_c = 1.0;          //!< [s] 加速度で引き戻す時定数。長いほどジャイロを信じる
  double gyro_lpf_hz = 30.0;   //!< [Hz] 角速度の 1 次 LPF。0 で素通し
  //! |f| が g からこの割合だけ外れたら加速度を全く使わない（その間は線形に重みを落とす）
  double accel_band = 0.25;
  double bias_tau = 2.0;       //!< [s] 静止中にバイアスを追う時定数
  double bias_max = 0.02;      //!< [rad/s] バイアスの上限（各軸）
  double rest_gyro = 0.05;     //!< [rad/s] これより遅ければ静止とみなす
  double rest_time = 0.5;      //!< [s] 静止がこれだけ続いたらバイアスを取り始める
  double avg_time = 1.0;       //!< [s] zero() が使う平均の時定数
  //! [rad] zero() が直してよい最大の角。これより傾いた読みは「立っていない」とみなして断る
  double zero_max = 25.0 * M_PI / 180.0;
  double nominal_hz = 200.0;   //!< 時刻が壊れているサンプルに使う周期
};

struct Attitude
{
  bool valid = false;          //!< 1 度でも初期化できたか
  double roll = 0.0;           //!< [rad] + で右へ倒れている
  double pitch = 0.0;          //!< [rad] + で前へ倒れている
  double gyro[3]{0.0, 0.0, 0.0};   //!< [rad/s] Σ_B。バイアスを引いて LPF を通したもの
  double stamp = 0.0;          //!< [s] 最後のサンプルの時刻（メッセージの header）
  double accel_weight = 0.0;   //!< 最後のサンプルで加速度をどれだけ信じたか [0, 1]
  bool at_rest = false;        //!< 静止判定（バイアスを取っている）
  std::uint64_t samples = 0;
};

class ImuAttitude
{
public:
  static constexpr double kGravity = 9.80665;

  /// 設定を入れる。mount_rpy が変わったときだけ推定をやり直す。
  void configure(const ImuOptions & opt);
  const ImuOptions & options() const {return opt_;}

  /// 1 サンプル。値は {O}（光学座標系）のまま渡す。
  ///   gyro_o   角速度 [rad/s]
  ///   accel_o  比力 [m/s^2]（静止時に上向き +g。RealSense の /camera/imu と同じ）
  void update(double stamp, const double gyro_o[3], const double accel_o[3]);

  const Attitude & attitude() const {return att_;}

  /// 計画上の重心の水平加速度 [m/s^2]（歩行計画の座標。x 前 / y 左）。
  /// 以後のサンプルで比力からこれを引いてから重力の向きとして使う。
  /// 機体は旋回しない（歩行計画の x = 機体の前）ので、胴体へは傾きだけで回す。
  /// 歩いていないときは 0 を入れる。
  void setAccelFeedforward(double ax, double ay)
  {
    ff_[0] = ax;
    ff_[1] = ay;
  }

  /// 今の姿勢をロール・ピッチ 0 にするよう mount_rpy を取り直す。
  /// 静止していない・データが足りない・直す角が zero_max を超える（寝ている・
  /// 吊られて傾いている）ときは何もせず false（理由は msg）。
  /// 成功したら新しい mount_rpy [rad] を out に書く。
  bool zero(double mount_rpy_out[3], std::string & msg);

  /// 推定を捨てる（次のサンプルで加速度から初期化し直す）。
  void reset();

  /// Σ_B のベクトルを {O} で表す。**テスト用**（合成したサンプルを作るため）。
  static rk::Vec3 opticalFromBody(const double mount_rpy[3], const rk::Vec3 & v_b);

private:
  void initFromAccel(const rk::Vec3 & f_b);
  void updateOutput();

  ImuOptions opt_;
  rk::Mat3 r_bo_ = mountMatrix(opt_.mount_rpy);
  static rk::Mat3 mountMatrix(const double mount_rpy[3]);

  double q_[4]{1.0, 0.0, 0.0, 0.0};   //!< R_WB の四元数 (w, x, y, z)
  bool init_ = false;
  double last_stamp_ = 0.0;
  rk::Vec3 bias_{};
  rk::Vec3 gyro_lpf_{};
  rk::Vec3 f_avg_{};                  //!< Σ_B の比力の平均（zero 用）
  double w_avg_ = 0.0;                //!< |ω| の平均（zero 用）
  double avg_fill_ = 0.0;             //!< 平均に入っている時間 [s]
  double rest_for_ = 0.0;
  double ff_[2]{0.0, 0.0};
  Attitude att_;
};

}  // namespace roboone_motion

#endif  // ROBOONE_MOTION__IMU_ATTITUDE_HPP_
