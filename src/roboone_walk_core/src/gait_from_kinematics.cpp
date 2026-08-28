// ホーム姿勢から LIPM の高さ z_c を割り出し、脚の到達域を IK で走査して
// 歩行パラメータ (gait.yaml) の目安を出す。
//
//   gait_from_kinematics [--bend 30] [--t-step 0.4] [--servo-speed 4.7] [--map]
//
// ホーム姿勢の定義 (2026-08-28、ユーザ指定):
//   T ポーズ (脚まっすぐ・足裏水平) から脚のピッチを bend [deg] 曲げる
//     = 股ピッチ ∓bend、膝 2·bend、足首ピッチ ±bend で、大腿・下腿がそれぞれ
//       鉛直から bend 傾き、胴体は直立・足裏は水平のまま (符号は FK で自動判定)
//   腕は id9 +50 deg (少し下ろす)、id10 +30 deg (少し前に曲げる)。脚の FK には効かない
//
// 「重心質点」は roboone_kinematics の Σ_B 原点 (股 3 軸の交点の高さ・左右中央) と
// みなす。IK の足裏座標はこの点からの相対位置で定義されているため。
// 実際の質量中心がこれより上にあるなら z_c をその分足す (ω が下がる方向)。
//
// 到達域の判定は 3 段階:
//   design : IK Ok + 関節リミット + サーボ側 Ok + 足首が設計可動域 (同時 ±15° の菱形)
//   mech   : IK Ok + 関節リミット + サーボ側 Ok (膝 4 節・足首ロッドが届き、クランク ±60°)
//   ik     : IK が解けるだけ
// 歩行のパラメータは design を基準に、mech を上限の参考として出す。

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "roboone_kinematics/ankle_config.hpp"
#include "roboone_kinematics/leg_config.hpp"
#include "roboone_kinematics/leg_kinematics.hpp"
#include "roboone_kinematics/leg_servo.hpp"
#include "roboone_walk_core/gait_params.hpp"

using namespace roboone_kinematics;  // NOLINT

namespace
{

constexpr double kDeg = M_PI / 180.0;

enum Level { NONE = 0, IK_ONLY = 1, MECH = 2, DESIGN = 3 };

struct Probe
{
  Level level = NONE;
  double th[kNumJoints] = {0, 0, 0, 0, 0, 0};
  double servo[kNumJoints] = {0, 0, 0, 0, 0, 0};
  double bend = 0.0;          // 膝の曲げ量 [rad]
  double ankleRoll = 0.0;     // Σ_B ロール [rad]
  double anklePitch = 0.0;    // Σ_B ピッチ [rad]
  LegServoStatus sst = LegServoStatus::Ok;
  IkStatus ist = IkStatus::Ok;
};

/// 足裏を (p, 水平) に置いたときの実現可能性 (右脚・Σ_B・mm)。
Probe probe(const LegServoParams & prm, const Vec3 & p)
{
  Probe r;
  Mat3 R;                                   // 単位行列 = 足裏水平
  r.ist = ik(prm.leg, p, R, r.th, /*clamp=*/false);
  if (r.ist != IkStatus::Ok) {return r;}
  r.level = IK_ONLY;
  // 関節リミット (Σ_B、右脚は AXIS_FLIP なし)。膝は曲げ量で見る
  r.bend = kneeBendFromLegAngle(prm.leg, r.th[KNEE]);
  for (std::size_t k = 0; k < kNumJoints; ++k) {
    const double v = (k == KNEE) ? r.bend : r.th[k];
    if (v < config::JOINT_LIMIT_LO_DEG[k] * kDeg || v > config::JOINT_LIMIT_HI_DEG[k] * kDeg) {
      return r;
    }
  }
  r.sst = legServoFromJoints(prm, r.th, r.servo);
  if (r.sst != LegServoStatus::Ok) {return r;}
  // 足首クランク ±60° (CRANK_LIMIT_DEG)。servo は σ·n·q + zero で n=1・zero=0 なので |q|
  const std::size_t ankleIdx[2] = {ANKLE_PITCH, ANKLE_ROLL};
  for (std::size_t k : ankleIdx) {
    if (std::abs(r.servo[k]) > ankle_config::CRANK_LIMIT_DEG[0][1] * kDeg) {return r;}
  }
  r.level = MECH;
  // 足首の設計可動域: 同時 ±15° の菱形 (ankle_config.hpp)。
  // enum の ANKLE_PITCH は Σ_B ではロール、ANKLE_ROLL はピッチ (leg_kinematics.hpp 冒頭)
  r.ankleRoll = r.th[ANKLE_PITCH];
  r.anklePitch = r.th[ANKLE_ROLL];
  const double lim = ankle_config::TH5_LIMIT_DEG[1] * kDeg;
  if (std::abs(r.ankleRoll) / lim + std::abs(r.anklePitch) / lim <= 1.0 + 1e-9) {
    r.level = DESIGN;
  }
  return r;
}

const char * levelName(Level l)
{
  switch (l) {
    case DESIGN: return "design";
    case MECH: return "mech";
    case IK_ONLY: return "ik";
    default: return "none";
  }
}

/// 原点から dir 方向 (mm 単位ベクトル) へ、level 以上が連続する最大距離 [mm]
double reach(const LegServoParams & prm, const Vec3 & origin, const Vec3 & dir,
             Level need, double maxDist = 300.0, double step = 1.0)
{
  double last = 0.0;
  for (double d = 0.0; d <= maxDist; d += step) {
    const Probe r = probe(prm, origin + dir * d);
    if (r.level < need) {break;}
    last = d;
  }
  return last;
}

double arg(int argc, char ** argv, const char * key, double def)
{
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::strcmp(argv[i], key) == 0) {return std::atof(argv[i + 1]);}
  }
  return def;
}

bool flag(int argc, char ** argv, const char * key)
{
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], key) == 0) {return true;}
  }
  return false;
}

}  // namespace

int main(int argc, char ** argv)
{
  const double bendDeg = arg(argc, argv, "--bend", 30.0);
  const double tStep = arg(argc, argv, "--t-step", 0.4);
  // サーボの無負荷角速度 [rad/s]。文書 §3.7 の STS3215 (0.222 s/60°) = 4.7 rad/s。
  // 実機は HLS 系なので要確認
  const double servoSpeed = arg(argc, argv, "--servo-speed", 4.7);
  const bool wantMap = flag(argc, argv, "--map");

  const LegServoParams prm = makeLegServoParams(Side::RIGHT);
  const Vec3 hip{config::HIP_X, config::HIP_Y, config::HIP_Z};   // Σ_B [mm]

  // ------------------------------------------------------------ ホーム姿勢
  // 符号は FK で決める: 足裏が水平 (R = I)、足がほぼ股の真下、
  // 膝が前に出る (o4 の x > 股の x) 組み合わせを採る
  const double b = bendDeg * kDeg;
  double th[kNumJoints] = {0, 0, 0, 0, 0, 0};
  Vec3 foot;
  bool found = false;
  for (int sh = -1; sh <= 1 && !found; sh += 2) {
    for (int sk = -1; sk <= 1 && !found; sk += 2) {
      for (int sa = -1; sa <= 1 && !found; sa += 2) {
        double t[kNumJoints] = {sh * b, 0.0, 0.0, sk * 2.0 * b, 0.0, sa * b};
        Vec3 p; Mat3 R;
        fk(prm.leg, t, p, R);
        double off = 0.0;
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {off += std::abs(R(i, j) - (i == j ? 1.0 : 0.0));}
        }
        Vec3 org[5];
        jointOrigins(prm.leg, t, org);
        // 足首 2 軸のオフセット L5 が下腿と一緒に傾くので、足は股の真下から数 mm ずれる
        if (off < 1e-9 && std::abs(p.x - hip.x) < 20.0 && org[1].x > hip.x + 1.0) {
          std::memcpy(th, t, sizeof(t));
          foot = p;
          found = true;
        }
      }
    }
  }
  if (!found) {
    std::fprintf(stderr, "ホーム姿勢の符号を決められない (bend=%g)\n", bendDeg);
    return 1;
  }
  const double zc_mm = -foot.z;
  const double zc = zc_mm / 1000.0;
  const Probe home = probe(prm, foot);

  std::printf("=== ホーム姿勢 (脚ピッチ %.0f deg 曲げ・腕 id9 +50 / id10 +30 は脚 FK に無関係) ===\n",
              bendDeg);
  std::printf("関節角 Σ_B [deg]: 股ピッチ %+.1f  股ロール %+.1f  股ヨー %+.1f  膝 %+.1f  "
              "足首(J5=ロール) %+.1f  足首(J6=ピッチ) %+.1f\n",
              th[0] / kDeg, th[1] / kDeg, th[2] / kDeg, th[3] / kDeg, th[4] / kDeg, th[5] / kDeg);
  std::printf("足裏中心 (右脚, Σ_B) = (%.1f, %.1f, %.1f) mm\n", foot.x, foot.y, foot.z);
  std::printf("z_c = 原点(股3軸の高さ・重心質点) から足裏まで = %.1f mm = %.4f m\n", zc_mm, zc);
  std::printf("  伸び切り %.1f mm の %.0f %%\n",
              config::L3 + config::L4 + config::L5 + config::L6,
              100.0 * zc_mm / (config::L3 + config::L4 + config::L5 + config::L6));
  std::printf("サーボ側の判定: %s  (膝サーボ %+.1f deg, 足首クランク q1 %+.1f / q2 %+.1f deg)\n",
              levelName(home.level), home.servo[KNEE] / kDeg,
              home.servo[ANKLE_PITCH] / kDeg, home.servo[ANKLE_ROLL] / kDeg);
  std::printf("  足首 ロール %+.1f / ピッチ %+.1f deg  設計可動域は同時 ±15 (菱形)、"
              "単軸ピッチ窓 ±55、ロール機構限界 ±35.8\n",
              home.ankleRoll / kDeg, home.anklePitch / kDeg);

  // ------------------------------------------------------------ 到達域
  const Vec3 fwd{1, 0, 0}, back{-1, 0, 0}, in{0, 1, 0}, out{0, -1, 0};   // 右脚: 内側 = +y
  std::printf("\n=== 到達域 (足裏水平・z = -z_c、股の真下から。右脚) [mm] ===\n");
  std::printf("%-8s %6s %6s %6s %6s\n", "判定", "前", "後", "内", "外");
  double rf[4], rm[4];
  {
    const Vec3 dirs[4] = {fwd, back, in, out};
    for (int i = 0; i < 4; ++i) {
      rf[i] = reach(prm, foot, dirs[i], DESIGN);
      rm[i] = reach(prm, foot, dirs[i], MECH);
    }
    std::printf("%-8s %6.0f %6.0f %6.0f %6.0f\n", "design", rf[0], rf[1], rf[2], rf[3]);
    std::printf("%-8s %6.0f %6.0f %6.0f %6.0f\n", "mech", rm[0], rm[1], rm[2], rm[3]);
  }
  // 遊脚の頂点 (z_c - h_sw) でも前後に届くか
  roboone_walk_core::GaitParams gp;
  {
    const Vec3 apex = foot + Vec3{0, 0, gp.swing_height * 1000.0};
    std::printf("遊脚頂点 (z = -z_c + %.0f mm): design 前 %.0f 後 %.0f / mech 前 %.0f 後 %.0f\n",
                gp.swing_height * 1000.0,
                reach(prm, apex, fwd, DESIGN), reach(prm, apex, back, DESIGN),
                reach(prm, apex, fwd, MECH), reach(prm, apex, back, MECH));
  }

  if (wantMap) {
    std::printf("\n到達域マップ (行 = 前後 dx、列 = 左右 dy。# design  + mech  . ik のみ  空白 不可)\n");
    std::printf("        dy:");
    for (double dy = -80; dy <= 80; dy += 10) {std::printf("%3.0f", dy / 10);}
    std::printf("  [cm]\n");
    for (double dx = 200; dx >= -200; dx -= 20) {
      std::printf("dx %+4.0f mm ", dx);
      for (double dy = -80; dy <= 80; dy += 10) {
        const Probe r = probe(prm, foot + Vec3{dx, dy, 0});
        const char c = r.level == DESIGN ? '#' : r.level == MECH ? '+' : r.level == IK_ONLY ? '.' : ' ';
        std::printf("  %c", c);
      }
      std::printf("\n");
    }
  }

  // ------------------------------------------------------ 歩行パラメータの目安
  const double W = 2.0 * std::abs(config::HIP_Y) / 1000.0;
  const double omega = std::sqrt(gp.gravity / zc);
  const double ewt = std::exp(omega * tStep);
  // 着地の瞬間、骨盤は両足のほぼ中央 → 各足は股の真下から ±Lx/2。
  // さらに式 (10) の補正とクランプ (±step_clamp_x) のぶんを余裕として引く
  const double margin = 0.85;
  const double lxDesign = 2.0 * margin * std::min(rf[0], rf[1]) / 1000.0 - gp.step_clamp_x;
  const double lxMech = 2.0 * margin * std::min(rm[0], rm[1]) / 1000.0 - gp.step_clamp_x;
  // 横は着地時に内側へ Ly 寄る (支持足から W - Ly)。内側到達域と内側クランプで決める
  const double lyDesign = margin * rf[2] / 1000.0 - gp.step_clamp_in;
  const double lyMech = margin * rm[2] / 1000.0 - gp.step_clamp_in;
  // 文書 §3.7 (4): 股・膝の角速度 ≈ 1.875 Lx/(T l_th) をサーボ無負荷速度の半分以下に
  // → T を固定すると Lx の上限になる
  const double lth = config::L3 / 1000.0;
  const double lxSpeed = 0.5 * servoSpeed * lth * tStep / 1.875;
  const double vxD = std::max(0.0, std::min(lxDesign, lxSpeed)) / tStep;
  const double vxM = std::max(0.0, std::min(lxMech, lxSpeed)) / tStep;
  const double vyD = std::max(0.0, lyDesign) / tStep, vyM = std::max(0.0, lyMech) / tStep;
  // 純 FF の安定条件 (walk_core_spec §6): a T² (1 + 1/(e^{ωT}∓1)) < クランプ
  const double axMax = gp.step_clamp_x / (tStep * tStep * (1.0 + 1.0 / (ewt - 1.0)));
  const double ayMax = gp.step_clamp_in / (tStep * tStep * (1.0 + 1.0 / (ewt + 1.0)));
  const double by = W / (ewt + 1.0);
  const double t0 = std::log(2.0 - 2.0 / (ewt + 1.0)) / omega;

  std::printf("\n=== 歩行パラメータの目安 (T = %.2f s) ===\n", tStep);
  std::printf("z_c            %.3f m   (ω = %.2f rad/s, e^{ωT} = %.1f)\n", zc, omega, ewt);
  std::printf("W (足間隔)     %.4f m   = 股間隔 2 x %.1f mm。walk_core の既定 0.08 とは食い違うので合わせる\n",
              W, std::abs(config::HIP_Y));
  std::printf("Lx 上限        design %.3f m / mech %.3f m   (到達域 x %.2f − クランプ %.3f)\n",
              lxDesign, lxMech, margin, gp.step_clamp_x);
  std::printf("Ly 上限        design %.3f m / mech %.3f m\n", lyDesign, lyMech);
  std::printf("v_max          design (%.3f, %.3f) / mech (%.3f, %.3f) m/s\n", vxD, vyD, vxM, vyM);
  std::printf("Lx 速度上限    %.3f m  (T=%.2f でサーボ無負荷 %.1f rad/s の半分以下に収める条件)\n",
              lxSpeed, tStep, servoSpeed);
  std::printf("a_max 上限     (%.2f, %.2f) m/s²  (純 FF の安定条件。余裕を見て 6 割程度に)\n",
              axMax, ayMax);
  std::printf("横の DCM 幅 b_y %.1f mm、押し出し時間 T0 %.3f s (start_pushoff_max %.2f の内側か確認)\n",
              by * 1000.0, t0, gp.start_pushoff_max);

  const bool useDesign = lxDesign > 0.0 && lyDesign > 0.0;
  const double vx = useDesign ? vxD : vxM, vy = useDesign ? vyD : vyM;
  std::printf("\n--- gait.yaml 案 (%s 基準%s) ---\n", useDesign ? "design" : "mech",
              useDesign ? "" : "。この姿勢では足首が設計可動域の外なので機構限界で決めた");
  std::printf("z_c: %.3f\nt_step: %.2f\nfoot_spacing: %.4f\n", zc, tStep, W);
  std::printf("v_max: [%.2f, %.2f]\na_max: [%.2f, %.2f]\n",
              std::floor(vx * 100) / 100, std::floor(vy * 100) / 100,
              std::floor(axMax * 0.6 * 100) / 100, std::floor(ayMax * 0.6 * 100) / 100);
  return 0;
}
