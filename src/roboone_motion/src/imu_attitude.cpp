#include "roboone_motion/imu_attitude.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace roboone_motion
{

namespace
{

constexpr double kR2D = 180.0 / M_PI;

double clamp(double v, double lo, double hi) {return v < lo ? lo : (v > hi ? hi : v);}

/// {O} -> {N}（軸の並べ替えだけ）。x_n = z_o, y_n = -x_o, z_n = -y_o
rk::Vec3 nominalFromOptical(const rk::Vec3 & o) {return {o.z, -o.x, -o.y};}

/// {N} -> {O}。上の逆。
rk::Vec3 opticalFromNominal(const rk::Vec3 & n) {return {-n.y, -n.z, n.x};}

/// 四元数 (w, x, y, z) -> 回転行列。
rk::Mat3 matFromQuat(const double q[4])
{
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  rk::Mat3 r;
  r(0, 0) = 1 - 2 * (y * y + z * z);
  r(0, 1) = 2 * (x * y - w * z);
  r(0, 2) = 2 * (x * z + w * y);
  r(1, 0) = 2 * (x * y + w * z);
  r(1, 1) = 1 - 2 * (x * x + z * z);
  r(1, 2) = 2 * (y * z - w * x);
  r(2, 0) = 2 * (x * z - w * y);
  r(2, 1) = 2 * (y * z + w * x);
  r(2, 2) = 1 - 2 * (x * x + y * y);
  return r;
}

/// RPY（R = rotZ * rotY * rotX）-> 四元数。
void quatFromRpy(double roll, double pitch, double yaw, double q[4])
{
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  q[0] = cy * cp * cr + sy * sp * sr;
  q[1] = cy * cp * sr - sy * sp * cr;
  q[2] = cy * sp * cr + sy * cp * sr;
  q[3] = sy * cp * cr - cy * sp * sr;
}

/// q <- q ⊗ exp(v)。v は機体側（右から掛ける）の回転ベクトル [rad]。
void quatIntegrate(double q[4], const rk::Vec3 & v)
{
  const double a = v.norm();
  double d[4]{1.0, 0.0, 0.0, 0.0};
  if (a > 1e-12) {
    const double s = std::sin(a / 2) / a;
    d[0] = std::cos(a / 2);
    d[1] = v.x * s;
    d[2] = v.y * s;
    d[3] = v.z * s;
  }
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  double r[4] = {
    w * d[0] - x * d[1] - y * d[2] - z * d[3],
    w * d[1] + x * d[0] + y * d[3] - z * d[2],
    w * d[2] - x * d[3] + y * d[0] + z * d[1],
    w * d[3] + x * d[2] - y * d[1] + z * d[0]};
  const double n = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]);
  for (int i = 0; i < 4; ++i) {
    q[i] = r[i] / n;
  }
}

/// 単位ベクトル a を +z に重ねる最小の回転（水平軸まわり）。
rk::Mat3 rotationTakingToUp(const rk::Vec3 & a)
{
  const rk::Vec3 z{0.0, 0.0, 1.0};
  const rk::Vec3 axis = a.cross(z);
  const double s = axis.norm();
  const double c = a.dot(z);
  if (s < 1e-12) {return rk::Mat3{};}
  const rk::Vec3 k = axis * (1.0 / s);
  const double ang = std::atan2(s, c);
  const double cs = std::cos(ang), sn = std::sin(ang), v = 1.0 - cs;
  rk::Mat3 r;
  r(0, 0) = cs + k.x * k.x * v;
  r(0, 1) = k.x * k.y * v - k.z * sn;
  r(0, 2) = k.x * k.z * v + k.y * sn;
  r(1, 0) = k.y * k.x * v + k.z * sn;
  r(1, 1) = cs + k.y * k.y * v;
  r(1, 2) = k.y * k.z * v - k.x * sn;
  r(2, 0) = k.z * k.x * v - k.y * sn;
  r(2, 1) = k.z * k.y * v + k.x * sn;
  r(2, 2) = cs + k.z * k.z * v;
  return r;
}

}  // namespace

rk::Mat3 ImuAttitude::mountMatrix(const double mount_rpy[3])
{
  // R_BO = Rrpy(mount) · P_NO。P_NO は列ごとに書くより、基底ベクトルを通して作る。
  const rk::Mat3 m = matFromRpy(mount_rpy);
  rk::Mat3 r;
  const rk::Vec3 e[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  for (int j = 0; j < 3; ++j) {
    const rk::Vec3 col = m * nominalFromOptical(e[j]);
    r(0, j) = col.x;
    r(1, j) = col.y;
    r(2, j) = col.z;
  }
  return r;
}

rk::Vec3 ImuAttitude::opticalFromBody(const double mount_rpy[3], const rk::Vec3 & v_b)
{
  return opticalFromNominal(matFromRpy(mount_rpy).mulT(v_b));
}

void ImuAttitude::configure(const ImuOptions & opt)
{
  // deg <-> rad の往復で末尾の桁がずれるので、厳密一致では比べない
  // （zero() の結果をパラメータへ書き戻したときに推定をやり直さないため）
  bool mount_changed = false;
  for (int k = 0; k < 3; ++k) {
    mount_changed = mount_changed || std::abs(opt.mount_rpy[k] - opt_.mount_rpy[k]) > 1e-9;
  }
  opt_ = opt;
  if (mount_changed) {
    r_bo_ = mountMatrix(opt_.mount_rpy);
    reset();
  }
}

void ImuAttitude::reset()
{
  init_ = false;
  bias_ = rk::Vec3{};
  gyro_lpf_ = rk::Vec3{};
  f_avg_ = rk::Vec3{};
  w_avg_ = 0.0;
  avg_fill_ = 0.0;
  rest_for_ = 0.0;
  att_ = Attitude{};
}

void ImuAttitude::initFromAccel(const rk::Vec3 & f_b)
{
  const double roll = std::atan2(f_b.y, f_b.z);
  const double pitch = std::atan2(-f_b.x, std::sqrt(f_b.y * f_b.y + f_b.z * f_b.z));
  quatFromRpy(roll, pitch, 0.0, q_);
  init_ = true;
}

void ImuAttitude::update(double stamp, const double gyro_o[3], const double accel_o[3])
{
  const rk::Vec3 w_b = r_bo_ * rk::Vec3{gyro_o[0], gyro_o[1], gyro_o[2]};
  const rk::Vec3 f_b = r_bo_ * rk::Vec3{accel_o[0], accel_o[1], accel_o[2]};
  const double fn = f_b.norm();
  if (!std::isfinite(fn) || !std::isfinite(w_b.norm()) || fn < 1e-6) {return;}

  double dt = stamp - last_stamp_;
  const bool gap = init_ && dt > 0.5;
  if (dt <= 0.0 || dt > 0.1) {dt = 1.0 / opt_.nominal_hz;}
  last_stamp_ = stamp;
  if (!init_ || gap) {
    // 最初の 1 本か、長く途切れたあと。積分の続きにせず加速度から取り直す
    initFromAccel(f_b);
    f_avg_ = f_b;
    avg_fill_ = 0.0;
    gyro_lpf_ = w_b - bias_;
  }

  // 計画上の加速度を引いた比力を、重力の向きとして使う（ヘッダ「推定の中身」）。
  // 胴体へは直前の推定の傾きだけで回す（ヨーは使わない）
  rk::Vec3 f_g = f_b;
  if (ff_[0] != 0.0 || ff_[1] != 0.0) {
    const double rp[3] = {att_.roll, att_.pitch, 0.0};
    f_g = f_b - matFromRpy(rp).mulT(rk::Vec3{ff_[0], ff_[1], 0.0});
  }
  const double fg = f_g.norm();

  // 加速度をどれだけ信じるか。g から外れるほど重力の方向として怪しい
  const double band = std::max(1e-3, opt_.accel_band);
  const double weight =
    (fg > 1e-6) ? clamp(1.0 - std::abs(fg - kGravity) / (band * kGravity), 0.0, 1.0) : 0.0;

  // 静止判定とバイアス
  const rk::Vec3 w_c0 = w_b - bias_;
  if (w_c0.norm() < opt_.rest_gyro && weight > 0.9) {
    rest_for_ += dt;
  } else {
    rest_for_ = 0.0;
  }
  const bool at_rest = rest_for_ >= opt_.rest_time;
  if (at_rest && opt_.bias_tau > 0.0) {
    const double k = std::min(1.0, dt / opt_.bias_tau);
    bias_ = bias_ + (w_b - bias_) * k;
    bias_.x = clamp(bias_.x, -opt_.bias_max, opt_.bias_max);
    bias_.y = clamp(bias_.y, -opt_.bias_max, opt_.bias_max);
    bias_.z = clamp(bias_.z, -opt_.bias_max, opt_.bias_max);
  }
  const rk::Vec3 w_c = w_b - bias_;

  // 相補フィルタ。v = 推定の鉛直（Σ_B）、a = 測った鉛直。e = a × v を足すと
  // 推定の鉛直が測った鉛直へ寄る（機体側から掛ける回転なので）。
  const rk::Mat3 r_wb = matFromQuat(q_);
  const rk::Vec3 v{r_wb(2, 0), r_wb(2, 1), r_wb(2, 2)};
  const rk::Vec3 a = (fg > 1e-6) ? f_g * (1.0 / fg) : v;
  const double kp = (opt_.tau_c > 1e-6) ? weight / opt_.tau_c : 0.0;
  quatIntegrate(q_, (w_c + a.cross(v) * kp) * dt);

  // 角速度の LPF（減衰項の入力。推定は通さない）
  double alpha = 1.0;
  if (opt_.gyro_lpf_hz > 0.0) {
    const double rc = 1.0 / (2.0 * M_PI * opt_.gyro_lpf_hz);
    alpha = dt / (dt + rc);
  }
  gyro_lpf_ = gyro_lpf_ + (w_c - gyro_lpf_) * alpha;

  // zero() 用の平均
  const double ka = (opt_.avg_time > 1e-6) ? std::min(1.0, dt / opt_.avg_time) : 1.0;
  f_avg_ = f_avg_ + (f_b - f_avg_) * ka;
  w_avg_ += (w_c.norm() - w_avg_) * ka;
  avg_fill_ += dt;

  att_.stamp = stamp;
  att_.accel_weight = weight;
  att_.at_rest = at_rest;
  ++att_.samples;
  updateOutput();
}

void ImuAttitude::updateOutput()
{
  // 式 (24)。R_WB の 3 行目だけを使うのでヨーに依らない
  const rk::Mat3 r = matFromQuat(q_);
  att_.pitch = std::atan2(-r(2, 0), std::sqrt(r(2, 1) * r(2, 1) + r(2, 2) * r(2, 2)));
  att_.roll = std::atan2(r(2, 1), r(2, 2));
  att_.gyro[0] = gyro_lpf_.x;
  att_.gyro[1] = gyro_lpf_.y;
  att_.gyro[2] = gyro_lpf_.z;
  att_.valid = init_;
}

bool ImuAttitude::zero(double mount_rpy_out[3], std::string & msg)
{
  char buf[512];
  if (!init_ || avg_fill_ < 2.0 * opt_.avg_time) {
    std::snprintf(
      buf, sizeof(buf), "IMU のサンプルが足りない (%.1fs 分。%.1fs 要る)",
      avg_fill_, 2.0 * opt_.avg_time);
    msg = buf;
    return false;
  }
  if (w_avg_ > opt_.rest_gyro) {
    std::snprintf(
      buf, sizeof(buf), "機体が動いている (平均角速度 %.3f rad/s > %.3f)。静止させてからやり直す",
      w_avg_, opt_.rest_gyro);
    msg = buf;
    return false;
  }
  const double fn = f_avg_.norm();
  if (std::abs(fn - kGravity) > 0.15 * kGravity) {
    std::snprintf(
      buf, sizeof(buf), "加速度の平均が重力と合わない (|f| = %.2f m/s^2)。静止させてからやり直す",
      fn);
    msg = buf;
    return false;
  }

  const double before_roll = att_.roll, before_pitch = att_.pitch;
  // 平均の比力を真上に向ける回転を、取り付けの回転の前に掛ける
  const rk::Vec3 up = f_avg_ * (1.0 / fn);
  const double tilt = std::acos(clamp(up.z, -1.0, 1.0));
  if (tilt > opt_.zero_max) {
    std::snprintf(
      buf, sizeof(buf),
      "今の読みが %.1f deg 傾いている (roll %.1f / pitch %.1f。直してよいのは %.0f deg まで)。"
      "機体をホーム姿勢で立たせてからやり直す。取り付けそのものが違うなら"
      " imu.mount_rpy_deg を先に直す",
      tilt * kR2D, before_roll * kR2D, before_pitch * kR2D, opt_.zero_max * kR2D);
    msg = buf;
    return false;
  }
  const rk::Mat3 rc = rotationTakingToUp(up);
  const rk::Mat3 m_new = rc * matFromRpy(opt_.mount_rpy);
  rpyFromMat(m_new, opt_.mount_rpy);
  r_bo_ = mountMatrix(opt_.mount_rpy);

  // Σ_B で持っていた量を新しい取り付けへ載せ替える
  f_avg_ = rc * f_avg_;
  bias_ = rc * bias_;
  gyro_lpf_ = rc * gyro_lpf_;
  initFromAccel(f_avg_);
  rest_for_ = 0.0;
  updateOutput();

  for (int k = 0; k < 3; ++k) {
    mount_rpy_out[k] = opt_.mount_rpy[k];
  }
  std::snprintf(
    buf, sizeof(buf),
    "零点を取った。直前の読み roll %.2f / pitch %.2f deg を 0 にした。"
    "mount_rpy_deg = [%.3f, %.3f, %.3f]",
    before_roll * kR2D, before_pitch * kR2D,
    opt_.mount_rpy[0] * kR2D, opt_.mount_rpy[1] * kR2D, opt_.mount_rpy[2] * kR2D);
  msg = buf;
  return true;
}

}  // namespace roboone_motion
