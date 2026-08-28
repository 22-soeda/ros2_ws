// 片脚 FK/IK の自己検算。
//   ros2 run roboone_kinematics leg_selftest [-n 姿勢数] [--seed S]
//
// docs/脚IK導出.tex §8 の検算を再現しつつ、x 成分を入れた拡張分
// （導出 [1] の不変性、a = 0 での文書との一致）を足してある。
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "roboone_kinematics/leg_kinematics.hpp"

using namespace roboone_kinematics;

namespace
{

int g_failures = 0;

void check(bool cond, const char * what)
{
  if (!cond) {
    std::printf("           *** 不一致: %s\n", what);
    ++g_failures;
  }
}

constexpr double kDeg = 180.0 / M_PI;

double maxAbsDiff(const Vec3 & a, const Vec3 & b)
{
  return std::max({std::fabs(a.x - b.x), std::fabs(a.y - b.y), std::fabs(a.z - b.z)});
}

double maxAbsDiff(const Mat3 & a, const Mat3 & b)
{
  double w = 0.0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {w = std::max(w, std::fabs(a(i, j) - b(i, j)));}
  }
  return w;
}

/// 角度差を (-π, π] に畳んだ絶対値。
double angleDiff(double x, double y)
{
  return std::fabs(std::atan2(std::sin(x - y), std::cos(x - y)));
}

/// 可動域内の一様乱数姿勢。いったん文書の符号（Σ_S）で作ってから、
/// (X-swap) と AXIS_FLIP を適用した公開符号（Σ_B。fk/ik が受け取る形）に直す。
void randomTheta(const LegParams & prm, std::mt19937_64 & rng, double th[kNumJoints])
{
  double internal[kNumJoints];
  for (std::size_t k = 0; k < kNumJoints; ++k) {
    std::uniform_real_distribution<double> d(
      config::JOINT_LIMIT_LO_DEG[k] / kDeg, config::JOINT_LIMIT_HI_DEG[k] / kDeg);
    internal[k] = d(rng);
  }
  internal[KNEE] = prm.sigma * internal[KNEE] + prm.phi;
  toSolverAngles(prm, internal, th);
}

// ---------------------------------------------------------------------------
void checkZeroPose(const LegParams & prm)
{
  const double zero[kNumJoints] = {0, 0, 0, 0, 0, 0};
  Vec3 p; Mat3 R;
  fk(prm, zero, p, R);
  // 全ての回転が I なのでリンクベクトルの単純和になる（Σ_S の和を Σ_B へ）
  const Vec3 want = fromSolver(prm.p0 + prm.p3 + prm.p4 + prm.p5 + prm.p6);
  const double ep = maxAbsDiff(p, want);
  const double eR = maxAbsDiff(R, Mat3{});
  std::printf("  ゼロ姿勢: p = (%.3f, %.3f, %.3f)  期待 p0+Σp_k = (%.3f, %.3f, %.3f)\n",
    p.x, p.y, p.z, want.x, want.y, want.z);
  std::printf("           位置誤差 %.2e mm / 姿勢誤差 %.2e\n", ep, eR);
  check(ep < 1e-12 && eR < 1e-12, "ゼロ姿勢");
}

void checkRoundtrip(const LegParams & prm, int n, std::uint64_t seed, const char * label)
{
  std::mt19937_64 rng(seed);
  double mp = 0.0, mR = 0.0, mth = 0.0;
  int nbad = 0;
  for (int i = 0; i < n; ++i) {
    double th[kNumJoints];
    randomTheta(prm, rng, th);
    Vec3 p; Mat3 Rm;
    fk(prm, th, p, Rm);

    double th2[kNumJoints];
    if (ik(prm, p, Rm, th2, /*clamp=*/false) != IkStatus::Ok) {++nbad; continue;}

    Vec3 p2; Mat3 R2;
    fk(prm, th2, p2, R2);
    mp = std::max(mp, maxAbsDiff(p2, p));
    mR = std::max(mR, maxAbsDiff(R2, Rm));
    for (std::size_t k = 0; k < kNumJoints; ++k) {mth = std::max(mth, angleDiff(th2[k], th[k]));}
  }
  std::printf("  FK(IK(FK(θ))) 比較 %d 姿勢%s:\n", n, label);
  std::printf("           位置の最大誤差   %.2e mm\n", mp);
  std::printf("           姿勢行列の最大誤差 %.2e\n", mR);
  std::printf("           関節角の最大誤差 %.2e deg\n", mth * kDeg);
  std::printf("           解けなかった数   %d 件\n", nbad);
  check(mp < 1e-9, "往復の位置");
  check(mR < 1e-12, "往復の姿勢");
  check(mth < 1e-9, "往復の関節角");
  check(nbad == 0, "可動域内で解けない姿勢がある");
}

/// 導出 [3][6][7] の成分式を FK の行列積と突き合わせる。
void checkClosedForms(LegParams prm, int n, std::uint64_t seed)
{
  // (X-3)〜(X-5) は p6 に依らない（r は o3 -> o6）が、念のため既定の p6 で回す
  std::mt19937_64 rng(seed);
  double wR123 = 0, wR456 = 0, wR = 0, wX8 = 0;
  for (int i = 0; i < n; ++i) {
    double th[kNumJoints];
    randomTheta(prm, rng, th);
    double t[kNumJoints];
    toSolverAngles(prm, th, t);   // 文書の成分式は Σ_S・内部符号

    const double c1 = std::cos(t[0]), s1 = std::sin(t[0]);
    const double c2 = std::cos(t[1]), s2 = std::sin(t[1]);
    const double c3 = std::cos(t[2]), s3 = std::sin(t[2]);
    const double c4 = std::cos(t[3]), s4 = std::sin(t[3]);
    const double c5 = std::cos(t[4]), s5 = std::sin(t[4]);
    const double c6 = std::cos(t[5]), s6 = std::sin(t[5]);

    // (FK-3) 相当。股は実機に合わせて Rx(θ1)·Ry(θ2)·Rz(θ3)（文書は Ry·Rx·Rz）
    const Mat3 R123doc{{{c2 * c3, -c2 * s3, s2},
                        {c1 * s3 + s1 * s2 * c3, c1 * c3 - s1 * s2 * s3, -s1 * c2},
                        {s1 * s3 - c1 * s2 * c3, s1 * c3 + c1 * s2 * s3, c1 * c2}}};
    wR123 = std::max(wR123, maxAbsDiff(R123doc, rotX(t[0]) * rotY(t[1]) * rotZ(t[2])));

    // (FK-5)
    const Mat3 R456doc{{{c5, s5 * s6, s5 * c6},
                        {s4 * s5, c4 * c6 - s4 * c5 * s6, -c4 * s6 - s4 * c5 * c6},
                        {-c4 * s5, s4 * c6 + c4 * c5 * s6, -s4 * s6 + c4 * c5 * c6}}};
    wR456 = std::max(wR456, maxAbsDiff(R456doc, rotX(t[3]) * rotY(t[4]) * rotX(t[5])));

    // (X-1)〜(X-5): r の成分式
    const double t4e = t[3] - prm.phi;
    const double A = prm.l3e * std::cos(t4e) + prm.l4;
    const double B = prm.l3e * std::sin(t4e);
    const double V = prm.a * s5 - A * c5 - prm.l5;
    const Vec3 rdoc{prm.a * c5 + A * s5, -B * c6 + V * s6, B * s6 + V * c6};

    Vec3 p; Mat3 Rm;
    solver::fk(prm, t, p, Rm);
    const Vec3 rimpl = Rm.mulT(p - prm.p0) - prm.p6;
    wR = std::max(wR, maxAbsDiff(rdoc, rimpl));

    // (X-6)(X-8): K から A が戻ること
    const double K = rimpl.normSq() - prm.a * prm.a - prm.l3e * prm.l3e +
      prm.l4 * prm.l4 - prm.l5 * prm.l5;
    const double Aback = (K + 2.0 * prm.l5 * prm.a * s5) / (2.0 * (prm.l4 + prm.l5 * c5));
    wX8 = std::max(wX8, std::fabs(Aback - A));
  }
  std::printf("  (FK-3) R123   最大差 %.2e\n", wR123);
  std::printf("  (FK-5) R456   最大差 %.2e\n", wR456);
  std::printf("  (X-1..5) r    最大差 %.2e\n", wR);
  std::printf("  (X-6,8) A     最大差 %.2e\n", wX8);
  check(wR123 < 1e-13 && wR456 < 1e-13 && wR < 1e-10 && wX8 < 1e-9, "閉形式の照合");
}

/// 導出 [1]: a3 と a4 は和だけが効く。
void checkXSplitInvariance(int n, std::uint64_t seed)
{
  struct Split { double a3, a4; };
  const Split splits[] = {{11.0, -4.0}, {-20.0, 27.0}, {7.0, 0.0}, {0.0, 7.0}};

  LegParams base = makeLegParams(Side::RIGHT);
  base.a3 = splits[0].a3; base.a4 = splits[0].a4; base.b = 5.0; base.finalize();

  std::mt19937_64 rng(seed);
  double wp = 0.0, wR = 0.0, wth = 0.0;
  for (int i = 0; i < n; ++i) {
    double th[kNumJoints];
    randomTheta(base, rng, th);
    Vec3 p0v; Mat3 R0v;
    fk(base, th, p0v, R0v);

    for (const auto & s : splits) {
      LegParams prm = base;
      prm.a3 = s.a3; prm.a4 = s.a4; prm.finalize();
      Vec3 p; Mat3 Rm;
      fk(prm, th, p, Rm);
      wp = std::max(wp, maxAbsDiff(p, p0v));
      wR = std::max(wR, maxAbsDiff(Rm, R0v));

      double th2[kNumJoints];
      check(ik(prm, p, Rm, th2, false) == IkStatus::Ok, "分け方を変えた IK");
      for (std::size_t k = 0; k < kNumJoints; ++k) {wth = std::max(wth, angleDiff(th2[k], th[k]));}
    }
  }
  std::printf("  a3+a4 = %g を保ったまま 4 通りに分け直し x %d 姿勢:\n",
    splits[0].a3 + splits[0].a4, n);
  std::printf("           姿勢の差 位置 %.2e mm / 姿勢 %.2e  (0 であるべき)\n", wp, wR);
  std::printf("           往復の関節角最大誤差 %.2e deg\n", wth * kDeg);
  check(wp < 1e-12 && wR < 1e-14, "x の分け方で姿勢が変わってしまう");
  check(wth < 1e-9, "分け方を変えると往復しない");
}

/// a = 0 で文書の (IK-2)(IK-10) に戻ること。
void checkReduceToDoc(int n, std::uint64_t seed)
{
  LegParams prm = makeLegParams(Side::RIGHT);
  prm.a3 = 0.0; prm.a4 = 0.0; prm.b = 0.0; prm.finalize();

  std::mt19937_64 rng(seed);
  double wIk2 = 0.0, wIk10 = 0.0, wCos = 0.0;
  LegParams noL5 = prm;
  noL5.l5 = 0.0; noL5.finalize();

  for (int i = 0; i < n; ++i) {
    double th[kNumJoints];
    randomTheta(prm, rng, th);
    double t[kNumJoints];
    toSolverAngles(prm, th, t);

    Vec3 p; Mat3 Rm;
    solver::fk(prm, t, p, Rm);
    const Vec3 r = Rm.mulT(p - prm.p0) - prm.p6;

    const double A = prm.l3 * std::cos(t[3]) + prm.l4;
    const double B = prm.l3 * std::sin(t[3]);
    const double c5 = std::cos(t[4]), s5 = std::sin(t[4]);

    // (IK-2): r_x = A·sinθ5
    wIk2 = std::max(wIk2, std::fabs(r.x - A * s5));

    // (X-9) と文書 (IK-10) が同じ θ6 を出すこと
    const double V = -(A * c5 + prm.l5);
    const double t6new = std::atan2(V * r.y + B * r.z, V * r.z - B * r.y);
    const double Q = A * c5 + prm.l5;
    const double t6doc = std::atan2(B * r.z - Q * r.y, -B * r.y - Q * r.z);
    wIk10 = std::max(wIk10, angleDiff(t6new, t6doc));

    // ℓ5 = 0 なら cosθ4 が余弦定理そのもの（文書 §5.7）
    double th0[kNumJoints];
    randomTheta(noL5, rng, th0);
    double t0[kNumJoints];
    toSolverAngles(noL5, th0, t0);
    Vec3 p0v; Mat3 R0v;
    solver::fk(noL5, t0, p0v, R0v);
    const Vec3 r0 = R0v.mulT(p0v - noL5.p0) - noL5.p6;
    const double law = (r0.normSq() - noL5.l3e * noL5.l3e - noL5.l4 * noL5.l4) /
      (2.0 * noL5.l3e * noL5.l4);
    double soli[kNumJoints];
    check(solver::ik(noL5, p0v, R0v, soli, false) == IkStatus::Ok, "ℓ5=0 の IK");
    wCos = std::max(wCos, std::fabs(law - std::cos(soli[KNEE] - noL5.phi)));
  }
  std::printf("  a=0 で (IK-2) r_x = A·sinθ5 : 最大差 %.2e mm\n", wIk2);
  std::printf("  a=0 で (X-9) と (IK-10)     : 最大差 %.2e deg\n", wIk10 * kDeg);
  std::printf("  ℓ5=0 で cosθ4 と余弦定理     : 最大差 %.2e\n", wCos);
  check(wIk2 < 1e-11 && wIk10 < 1e-12 && wCos < 1e-12, "文書への帰着");
}

/// AXIS_FLIP 全 64 通りで、符号だけが変わり幾何は変わらないこと。
void checkAxisFlip(int n, std::uint64_t seed)
{
  LegParams base = makeLegParams(Side::RIGHT);
  base.a3 = 9.0; base.a4 = -3.0; base.b = 4.0; base.finalize();
  for (std::size_t k = 0; k < kNumJoints; ++k) {base.sign[k] = 1.0;}

  std::mt19937_64 rng(seed);
  std::vector<std::array<double, kNumJoints>> thetas(n);
  for (int i = 0; i < n; ++i) {randomTheta(base, rng, thetas[i].data());}

  double wPose = 0.0, wRound = 0.0;
  for (int mask = 0; mask < 64; ++mask) {
    LegParams prm = base;
    for (std::size_t k = 0; k < kNumJoints; ++k) {
      prm.sign[k] = ((mask >> k) & 1) ? -1.0 : 1.0;
    }
    for (const auto & ti : thetas) {
      double thExt[kNumJoints];
      applyFlip(prm, ti.data(), thExt);      // 同じ姿勢を外部符号で書いたもの

      Vec3 p0v, p1v; Mat3 R0v, R1v;
      fk(base, ti.data(), p0v, R0v);
      fk(prm, thExt, p1v, R1v);
      wPose = std::max({wPose, maxAbsDiff(p1v, p0v), maxAbsDiff(R1v, R0v)});

      double th2[kNumJoints];
      check(ik(prm, p1v, R1v, th2, false) == IkStatus::Ok, "flip 下の IK");
      for (std::size_t k = 0; k < kNumJoints; ++k) {
        wRound = std::max(wRound, angleDiff(th2[k], thExt[k]));
      }
    }
  }
  std::printf("  64 通りの AXIS_FLIP x %d 姿勢:\n", n);
  std::printf("           flip による姿勢の差 %.2e  (0 であるべき)\n", wPose);
  std::printf("           往復の関節角最大誤差 %.2e deg\n", wRound * kDeg);
  check(wPose < 1e-12, "flip が幾何を変えてしまっている");
  check(wRound < 1e-9, "flip 下で往復しない");
}

void checkUnreachable(const LegParams & prm)
{
  const double reach = prm.l3e + prm.l4 + prm.l5;
  // r_x^2 ~ ℓ3'^2 - ℓ4^2 + ℓ5^2 + a^2 なら K ~ 0 で (X-7) の |C| > |(P,Q)| を踏む
  const double rxCrit = std::sqrt(std::max(
    0.0, prm.l3e * prm.l3e - prm.l4 * prm.l4 + prm.l5 * prm.l5 + prm.a * prm.a));
  struct Case { const char * label; Vec3 p; IkStatus want; };
  const Vec3 hip = fromSolver(prm.p0);
  const Case cases[] = {
    {"遠すぎる", hip + Vec3{0, 0, -(reach + prm.l6 + 50.0)}, IkStatus::KneeOutOfRange},
    {"近すぎる", hip + Vec3{0, 0, -(prm.l6 + 1.0)}, IkStatus::NoBranch},
    // x_S 方向 = Σ_B の -y（右）へ遠ざける
    {"左右に遠い", hip + fromSolver(prm.p6) + Vec3{0, -rxCrit, 0}, IkStatus::AnkleOutOfRange},
  };
  const Mat3 I{};
  for (const auto & c : cases) {
    double th[kNumJoints];
    const IkStatus st = ik(prm, c.p, I, th, /*clamp=*/false);
    const auto name = [](IkStatus s) {
        return s == IkStatus::Ok ? "Ok (検出されなかった)" :
               s == IkStatus::AnkleOutOfRange ? "AnkleOutOfRange" :
               s == IkStatus::KneeOutOfRange ? "KneeOutOfRange" : "NoBranch";
      };
    std::printf("  %-16s -> %-15s (期待 %s)\n", c.label, name(st), name(c.want));
    check(st != IkStatus::Ok, "到達不能が検出されない");
    check(st == c.want, "到達不能の種類が期待と違う");

    // clamp=true なら最寄り姿勢が返る（NoBranch を除く）
    double thc[kNumJoints];
    const IkStatus stc = ik(prm, c.p, I, thc, /*clamp=*/true);
    if (stc != IkStatus::NoBranch) {
      bool finite = true;
      for (double v : thc) {finite = finite && std::isfinite(v);}
      check(finite, "clamp で有限でない角が出た");
    }
  }
}

/// 座標系の取り決めを数値で確かめる。
///   * ゼロ姿勢で o3 / o5 / o6 / 足裏中心が鉛直に並ぶ（実機の申告どおりか）
///   * 屈むと膝が前に出る（人型。KNEE_FORWARD = +1）
///   * 左右が y = 0 面の鏡像になっている
void checkBodyFrame(int n, std::uint64_t seed)
{
  const LegParams right = makeLegParams(Side::RIGHT);
  const LegParams left = makeLegParams(Side::LEFT);

  // --- ゼロ姿勢の関節位置（Σ_B） ---
  const double zero[kNumJoints] = {0, 0, 0, 0, 0, 0};
  Vec3 o[5];
  jointOrigins(right, zero, o);
  const char * nm[5] = {"o3 股中心", "o4 膝", "o5 足首 1", "o6 足首 2", "足裏中心"};
  std::printf("  ゼロ姿勢の関節位置 (Σ_B = x 前 / y 左 / z 上、右脚):\n");
  for (int k = 0; k < 5; ++k) {
    std::printf("           %-10s (%9.3f, %9.3f, %9.3f)\n", nm[k], o[k].x, o[k].y, o[k].z);
  }
  const double vx = std::max({std::fabs(o[0].x - o[2].x), std::fabs(o[0].x - o[3].x),
      std::fabs(o[0].x - o[4].x)});
  const double vy = std::max({std::fabs(o[0].y - o[2].y), std::fabs(o[0].y - o[3].y),
      std::fabs(o[0].y - o[4].y)});
  std::printf("           o3 / o5・o6 / 足裏中心 の水平ずれ %.2e mm\n", std::max(vx, vy));
  check(vx < 1e-9 && vy < 1e-9, "股中心・足首リンク・足裏中心が鉛直に並んでいない");

  // --- 膝の向き: 50 mm 屈んで膝が前に出るか ---
  const Vec3 hip = fromSolver(right.p0);
  const double stand = right.l3 + right.l4 + right.l5 + right.l6;
  const Vec3 target = hip + Vec3{0.0, 0.0, -(stand - 50.0)};
  const Mat3 I{};
  double th[kNumJoints];
  check(ik(right, target, I, th, /*clamp=*/false) == IkStatus::Ok, "屈み姿勢が解けない");
  jointOrigins(right, th, o);
  std::printf("  50 mm 屈む: θ1 = %+.2f deg, θ4 = %+.2f deg, 膝の前後位置 %+.2f mm\n",
    th[HIP_PITCH] * kDeg, th[KNEE] * kDeg, o[1].x - hip.x);
  check(th[KNEE] * config::KNEE_FORWARD > 0.0, "膝の符号が KNEE_FORWARD と合わない");
  check((o[1].x - hip.x) * config::KNEE_FORWARD > 0.0, "膝が前に出ない（逆関節になっている）");

  // --- 左右の鏡像 ---
  // y = 0 面の鏡映では Σ_B の Rx / Rz まわりの角（J2 股ロール, J3 股ヨー,
  // J5 足首）が反転し、Ry まわり（J1 股ピッチ, J4 膝, J6 足首）はそのまま。
  // 幾何だけを見たいので AXIS_FLIP は落としておく。
  LegParams r2 = right, l2 = left;
  for (std::size_t k = 0; k < kNumJoints; ++k) {r2.sign[k] = 1.0; l2.sign[k] = 1.0;}
  std::mt19937_64 rng(seed);
  double wp = 0.0, wR = 0.0;
  for (int i = 0; i < n; ++i) {
    double thr[kNumJoints];
    randomTheta(r2, rng, thr);
    const double thl[kNumJoints] = {thr[0], -thr[1], -thr[2], thr[3], -thr[4], thr[5]};
    Vec3 pr, pl;
    Mat3 Rr, Rl;
    fk(r2, thr, pr, Rr);
    fk(l2, thl, pl, Rl);
    Mat3 want = Rr;                       // M R M,  M = diag(1, -1, 1)
    want(0, 1) = -want(0, 1); want(1, 0) = -want(1, 0);
    want(1, 2) = -want(1, 2); want(2, 1) = -want(2, 1);
    wp = std::max(wp, maxAbsDiff(pl, Vec3{pr.x, -pr.y, pr.z}));
    wR = std::max(wR, maxAbsDiff(Rl, want));
  }
  std::printf("  左右の鏡像 x %d 姿勢: 位置 %.2e mm / 姿勢 %.2e  (0 であるべき)\n", n, wp, wR);
  check(wp < 1e-9 && wR < 1e-12, "左右が鏡像になっていない");
}

void benchmark(const LegParams & prm, int n)
{
  std::mt19937_64 rng(12345);
  std::vector<std::array<double, kNumJoints>> th(n);
  std::vector<Vec3> ps(n);
  std::vector<Mat3> Rs(n);
  for (int i = 0; i < n; ++i) {
    randomTheta(prm, rng, th[i].data());
    fk(prm, th[i].data(), ps[i], Rs[i]);
  }

  volatile double sink = 0.0;
  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) {
    Vec3 p; Mat3 R;
    fk(prm, th[i].data(), p, R);
    sink += p.x;
  }
  auto t1 = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) {
    double out[kNumJoints];
    ik(prm, ps[i], Rs[i], out, true);
    sink += out[0];
  }
  auto t2 = std::chrono::steady_clock::now();

  const double fkUs = std::chrono::duration<double, std::micro>(t1 - t0).count() / n;
  const double ikUs = std::chrono::duration<double, std::micro>(t2 - t1).count() / n;
  std::printf("  fk %.3f us/call   ik %.3f us/call   (%d 回平均)\n", fkUs, ikUs, n);
  std::printf("  両脚 fk+ik = %.3f us -> 200 Hz (5000 us) の %.4f %%\n",
    2.0 * (fkUs + ikUs), 2.0 * (fkUs + ikUs) / 5000.0 * 100.0);
}

}  // namespace

int main(int argc, char ** argv)
{
  int n = 20000;
  std::uint64_t seed = 0;
  for (int i = 1; i < argc; ++i) {
    if ((!std::strcmp(argv[i], "-n")) && i + 1 < argc) {n = std::atoi(argv[++i]);}
    else if ((!std::strcmp(argv[i], "--seed")) && i + 1 < argc) {
      seed = std::strtoull(argv[++i], nullptr, 10);
    }
  }

  std::printf("======================================================================\n");
  std::printf("片脚 FK / IK 自己検算  (C++ / docs/脚IK導出.tex + x 成分の拡張)\n");
  std::printf("======================================================================\n");

  for (const auto side : {Side::RIGHT, Side::LEFT}) {
    LegParams prm = makeLegParams(side);
    check(prm.valid(), "パラメータが前提を満たしていない");
    std::printf("\n[%s] ℓ3=%g ℓ4=%g ℓ5=%g ℓ6=%g  a3=%g a4=%g (a=%g) b=%g σ=%+d [Σ_S]\n",
      side == Side::RIGHT ? "right" : "left",
      prm.l3, prm.l4, prm.l5, prm.l6, prm.a3, prm.a4, prm.a, prm.b, prm.sigma);
    const Vec3 hipB = fromSolver(prm.p0);
    std::printf("       p0=(%g, %g, %g) [Σ_B]  flip={%d,%d,%d,%d,%d,%d}\n",
      hipB.x, hipB.y, hipB.z,
      prm.sign[0] < 0, prm.sign[1] < 0, prm.sign[2] < 0,
      prm.sign[3] < 0, prm.sign[4] < 0, prm.sign[5] < 0);
    checkZeroPose(prm);
    checkRoundtrip(prm, n, seed, "");
  }

  {
    std::printf("\n[x 成分あり  a3=11, a4=-4, b=7]\n");
    LegParams prm = makeLegParams(Side::RIGHT);
    prm.a3 = 11.0; prm.a4 = -4.0; prm.b = 7.0; prm.finalize();
    checkZeroPose(prm);
    checkRoundtrip(prm, n, seed + 1, "");

    std::printf("\n[文書・導出の閉形式との照合]\n");
    checkClosedForms(prm, std::min(n, 2000), seed + 2);
  }

  std::printf("\n[x の分け方の不変性 (導出 [1])]\n");
  checkXSplitInvariance(std::min(n, 2000), seed + 3);

  std::printf("\n[a = 0 で文書の式に戻ること]\n");
  checkReduceToDoc(std::min(n, 2000), seed + 4);

  {
    std::printf("\n[膝の分岐 σ = -1 (x 成分あり)]\n");
    LegParams prm = makeLegParams(Side::RIGHT);
    prm.a3 = -6.0; prm.a4 = 2.5; prm.b = -3.0; prm.sigma = -1; prm.finalize();
    checkRoundtrip(prm, std::min(n, 5000), seed + 5, "");
  }

  std::printf("\n[座標系 Σ_B と左右対称]\n");
  checkBodyFrame(std::min(n, 2000), seed + 7);

  std::printf("\n[軸の回転方向 AXIS_FLIP]\n");
  checkAxisFlip(std::min(n, 200), seed + 6);

  {
    std::printf("\n[到達不能の判定]\n");
    LegParams prm = makeLegParams(Side::RIGHT);
    prm.a3 = 11.0; prm.a4 = -4.0; prm.finalize();
    checkUnreachable(prm);
  }

  {
    std::printf("\n[速度]\n");
    LegParams prm = makeLegParams(Side::RIGHT);
    prm.a3 = 11.0; prm.a4 = -4.0; prm.b = 7.0; prm.finalize();
    benchmark(prm, std::max(n, 100000));
  }

  std::printf("\n======================================================================\n");
  if (g_failures == 0) {
    std::printf("すべて一致\n");
  } else {
    std::printf("不一致 %d 件\n", g_failures);
  }
  std::printf("======================================================================\n");
  return g_failures == 0 ? 0 : 1;
}
