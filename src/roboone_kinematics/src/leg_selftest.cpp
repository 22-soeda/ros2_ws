// 片脚 FK/IK の自己検算。
//   ros2 run roboone_kinematics leg_selftest [-n 姿勢数] [--seed S]
//
// 往復・成分式の照合・オフセットの分け方の不変性・a = 0 での簡約・AXIS_FLIP・
// 到達不能の判定・左右対称・座標系の向き（膝が前に出る）を見る。
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

/// 可動域内の一様乱数姿勢。内部角（膝は曲げ量）で作ってから、
/// AXIS_FLIP を適用した公開角（fk/ik が受け取る形）に直す。
void randomTheta(const LegParams & prm, std::mt19937_64 & rng, double th[kNumJoints])
{
  double internal[kNumJoints];
  for (std::size_t k = 0; k < kNumJoints; ++k) {
    std::uniform_real_distribution<double> d(
      config::JOINT_LIMIT_LO_DEG[k] / kDeg, config::JOINT_LIMIT_HI_DEG[k] / kDeg);
    internal[k] = d(rng);
  }
  internal[KNEE] = prm.sigma * internal[KNEE] - prm.phi;   // 曲げ量 -> θ4（θ4' = θ4 + φ）
  applyFlip(prm, internal, th);
}

// ---------------------------------------------------------------------------
void checkZeroPose(const LegParams & prm)
{
  const double zero[kNumJoints] = {0, 0, 0, 0, 0, 0};
  Vec3 p; Mat3 R;
  fk(prm, zero, p, R);
  // 全ての回転が I なのでリンクベクトルの単純和になる
  const Vec3 want = prm.p0 + prm.p3 + prm.p4 + prm.p5 + prm.p6;
  const double ep = maxAbsDiff(p, want);
  const double eR = maxAbsDiff(R, Mat3{});
  std::printf("  ゼロ姿勢: p = (%.3f, %.3f, %.3f)  期待 p0+Σp_k = (%.3f, %.3f, %.3f)\n",
    p.x, p.y, p.z, want.x, want.y, want.z);
  std::printf("           位置誤差 %.2e mm / 姿勢誤差 %.2e\n", ep, eR);
  check(ep < 1e-12 && eR < 1e-12, "ゼロ姿勢");
}

/// 往復。a = 0（実機）では解が一意なので関節角まで戻ることを要求する。
/// a ≠ 0 では (B-8) の 2 根が両方とも可動域内に落ちる姿勢があり、どちらも同じ足先
/// 姿勢を与える（IK の解が一意でない）。そこでは足先姿勢が戻ることだけを要求し、
/// 別の根を選んだ姿勢の数を出す。
void checkRoundtrip(const LegParams & prm, int n, std::uint64_t seed, const char * label)
{
  const bool unique = std::fabs(prm.a) < 1e-12;
  std::mt19937_64 rng(seed);
  double mp = 0.0, mR = 0.0, mth = 0.0;
  int nbad = 0, nbranch = 0;
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
    double d = 0.0;
    for (std::size_t k = 0; k < kNumJoints; ++k) {d = std::max(d, angleDiff(th2[k], th[k]));}
    if (d * kDeg > 1e-6) {++nbranch;} else {mth = std::max(mth, d);}
  }
  std::printf("  FK(IK(FK(θ))) 比較 %d 姿勢%s:\n", n, label);
  std::printf("           位置の最大誤差   %.2e mm\n", mp);
  std::printf("           姿勢行列の最大誤差 %.2e\n", mR);
  std::printf("           関節角の最大誤差 %.2e deg（同じ根に落ちた姿勢）\n", mth * kDeg);
  std::printf("           別の根に落ちた数 %d 件%s\n", nbranch,
    unique ? "" : "（a ≠ 0 では解が一意でない。足先姿勢は一致している）");
  std::printf("           解けなかった数   %d 件\n", nbad);
  check(mp < 1e-9, "往復の位置");
  check(mR < 1e-12, "往復の姿勢");
  check(mth < 1e-9, "往復の関節角");
  check(!unique || nbranch == 0, "a = 0 なのに別の根に落ちた");
  check(nbad == 0, "可動域内で解けない姿勢がある");
}

/// 導出 (B-1)〜(B-11) の成分式を FK の行列積と突き合わせる。
void checkClosedForms(LegParams prm, int n, std::uint64_t seed)
{
  // r の成分式は p6 に依らない（r は o3 -> o6）が、念のため既定の p6 で回す
  std::mt19937_64 rng(seed);
  double wR123 = 0, wR456 = 0, wR = 0, wB10 = 0, wB7 = 0, wB11 = 0;
  for (int i = 0; i < n; ++i) {
    double th[kNumJoints];
    randomTheta(prm, rng, th);
    double t[kNumJoints];
    applyFlip(prm, th, t);   // 成分式は内部角

    const double c1 = std::cos(t[0]), s1 = std::sin(t[0]);
    const double c2 = std::cos(t[1]), s2 = std::sin(t[1]);
    const double c3 = std::cos(t[2]), s3 = std::sin(t[2]);
    const double c5 = std::cos(t[4]), s5 = std::sin(t[4]);
    const double c6 = std::cos(t[5]), s6 = std::sin(t[5]);
    const double ca = std::cos(t[3] + t[4]), sa = std::sin(t[3] + t[4]);   // 膝 + 足首ピッチ

    // R123 = Ry(θ1)·Rx(θ2)·Rz(θ3) の成分（ik() の取り出しがこの形を前提にする）
    const Mat3 R123doc{{{c1 * c3 + s1 * s2 * s3, -c1 * s3 + s1 * s2 * c3, s1 * c2},
                        {c2 * s3, c2 * c3, -s2},
                        {-s1 * c3 + c1 * s2 * s3, s1 * s3 + c1 * s2 * c3, c1 * c2}}};
    wR123 = std::max(wR123, maxAbsDiff(R123doc, rotY(t[0]) * rotX(t[1]) * rotZ(t[2])));

    // R456 = Ry(θ4)·Ry(θ5)·Rx(θ6) = Ry(θ4+θ5)·Rx(θ6)。膝と足首ピッチは同軸
    const Mat3 R456doc{{{ca, sa * s6, sa * c6},
                        {0.0, c6, -s6},
                        {-sa, ca * s6, ca * c6}}};
    wR456 = std::max(wR456, maxAbsDiff(R456doc, rotY(t[3]) * rotY(t[4]) * rotX(t[5])));

    // (B-1)〜(B-5): r の成分式
    const double t4e = t[3] + prm.phi;
    const double A = prm.l3e * std::cos(t4e) + prm.l4;
    const double B = prm.l3e * std::sin(t4e);
    const double V = B * s5 - A * c5 - prm.l5;
    const Vec3 rdoc{B * c5 + A * s5, prm.a * c6 + V * s6, -prm.a * s6 + V * c6};

    Vec3 p; Mat3 Rm;
    solver::fk(prm, t, p, Rm);
    const Vec3 rimpl = Rm.mulT(p - prm.p0) - prm.p6;
    wR = std::max(wR, maxAbsDiff(rdoc, rimpl));

    // (B-7): r_y c6 - r_z s6 = a
    wB7 = std::max(wB7, std::fabs(rimpl.y * c6 - rimpl.z * s6 - prm.a));

    // (B-9)(B-10): θ6 が分かれば w が戻り、A が戻ること
    const double wz = rimpl.y * s6 + rimpl.z * c6;
    const double Aback = (rimpl.x * rimpl.x + (wz + prm.l5) * (wz + prm.l5) -
      prm.l3e * prm.l3e + prm.l4 * prm.l4) / (2.0 * prm.l4);
    wB10 = std::max(wB10, std::fabs(Aback - A));

    // (B-11): θ5 が戻ること
    const double t5back = std::atan2(wz + prm.l5, rimpl.x) - std::atan2(-A, B);
    wB11 = std::max(wB11, angleDiff(t5back, t[4]));
  }
  std::printf("  R123 成分式   最大差 %.2e\n", wR123);
  std::printf("  R456 成分式   最大差 %.2e\n", wR456);
  std::printf("  (B-1..5) r    最大差 %.2e\n", wR);
  std::printf("  (B-7) a       最大差 %.2e\n", wB7);
  std::printf("  (B-10) A      最大差 %.2e\n", wB10);
  std::printf("  (B-11) θ5     最大差 %.2e deg\n", wB11 * kDeg);
  check(wR123 < 1e-13 && wR456 < 1e-13 && wR < 1e-10 && wB7 < 1e-10 && wB10 < 1e-9 &&
    wB11 < 1e-9, "閉形式の照合");
}

/// 導出 [1]: a3 と a4（膝軸方向）は和だけが効く。
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

    // 分け方を変えても同じ根に落ちること（a は同じなので IK の 2 根も同じ）
    double thRef[kNumJoints];
    check(ik(base, p0v, R0v, thRef, false) == IkStatus::Ok, "分け方の基準の IK");
    for (const auto & s : splits) {
      LegParams prm = base;
      prm.a3 = s.a3; prm.a4 = s.a4; prm.finalize();
      Vec3 p; Mat3 Rm;
      fk(prm, th, p, Rm);
      wp = std::max(wp, maxAbsDiff(p, p0v));
      wR = std::max(wR, maxAbsDiff(Rm, R0v));

      double th2[kNumJoints];
      check(ik(prm, p, Rm, th2, false) == IkStatus::Ok, "分け方を変えた IK");
      for (std::size_t k = 0; k < kNumJoints; ++k) {wth = std::max(wth, angleDiff(th2[k], thRef[k]));}
    }
  }
  std::printf("  a3+a4 = %g を保ったまま 4 通りに分け直し x %d 姿勢:\n",
    splits[0].a3 + splits[0].a4, n);
  std::printf("           姿勢の差 位置 %.2e mm / 姿勢 %.2e  (0 であるべき)\n", wp, wR);
  std::printf("           IK の関節角の差   %.2e deg\n", wth * kDeg);
  check(wp < 1e-12 && wR < 1e-14, "オフセットの分け方で姿勢が変わってしまう");
  check(wth < 1e-9, "分け方を変えると IK が別の根に落ちる");
}

/// a = 0（実機）で式が簡単になること。
///   * 足首ロールが θ6 = atan2(-r_y, -r_z) で閉じる（(B-8) の V < 0 の枝）
///   * ℓ5 = 0 なら cosθ4 が余弦定理そのもの
void checkReduceToSimple(int n, std::uint64_t seed)
{
  LegParams prm = makeLegParams(Side::RIGHT);
  prm.a3 = 0.0; prm.a4 = 0.0; prm.b = 0.0; prm.finalize();

  std::mt19937_64 rng(seed);
  double wT6 = 0.0, wCos = 0.0;
  LegParams noL5 = prm;
  noL5.l5 = 0.0; noL5.finalize();

  for (int i = 0; i < n; ++i) {
    double th[kNumJoints];
    randomTheta(prm, rng, th);
    double t[kNumJoints];
    applyFlip(prm, th, t);

    Vec3 p; Mat3 Rm;
    solver::fk(prm, t, p, Rm);
    const Vec3 r = Rm.mulT(p - prm.p0) - prm.p6;

    // a = 0: (B-7) は r_y c6 = r_z s6、2 根は π 違うので |θ6| <= 90° の側が一意に決まる
    double sol[kNumJoints];
    check(solver::ik(prm, p, Rm, sol, false) == IkStatus::Ok, "a=0 の IK");
    const double t6 = sol[ANKLE_ROLL];
    wT6 = std::max(wT6, std::fabs(r.y * std::cos(t6) - r.z * std::sin(t6)) / std::hypot(r.y, r.z));
    check(std::fabs(t6) <= M_PI / 2 + 1e-12, "a=0 で |θ6| > 90° の根を選んだ");

    // ℓ5 = 0 なら cosθ4' が余弦定理そのもの
    double th0[kNumJoints];
    randomTheta(noL5, rng, th0);
    double t0[kNumJoints];
    applyFlip(noL5, th0, t0);
    Vec3 p0v; Mat3 R0v;
    solver::fk(noL5, t0, p0v, R0v);
    const Vec3 r0 = R0v.mulT(p0v - noL5.p0) - noL5.p6;
    const double law = (r0.normSq() - noL5.l3e * noL5.l3e - noL5.l4 * noL5.l4) /
      (2.0 * noL5.l3e * noL5.l4);
    double soli[kNumJoints];
    check(solver::ik(noL5, p0v, R0v, soli, false) == IkStatus::Ok, "ℓ5=0 の IK");
    wCos = std::max(wCos, std::fabs(law - std::cos(soli[KNEE] + noL5.phi)));
  }
  std::printf("  a=0 で r_y c6 = r_z s6 (B-7)  : 残差 %.2e\n", wT6);
  std::printf("  ℓ5=0 で cosθ4' と余弦定理      : 最大差 %.2e\n", wCos);
  check(wT6 < 1e-12 && wCos < 1e-12, "簡単な形への帰着");
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

      // flip を掛けても同じ根に落ちること（base の IK を flip したものと比べる。
      // a ≠ 0 なので元の姿勢そのものとは別の根に落ちる場合があり、それは幾何の性質）
      double thBase[kNumJoints], thWant[kNumJoints], th2[kNumJoints];
      check(ik(base, p0v, R0v, thBase, false) == IkStatus::Ok, "flip の基準の IK");
      applyFlip(prm, thBase, thWant);
      check(ik(prm, p1v, R1v, th2, false) == IkStatus::Ok, "flip 下の IK");
      for (std::size_t k = 0; k < kNumJoints; ++k) {
        wRound = std::max(wRound, angleDiff(th2[k], thWant[k]));
      }
    }
  }
  std::printf("  64 通りの AXIS_FLIP x %d 姿勢:\n", n);
  std::printf("           flip による姿勢の差 %.2e  (0 であるべき)\n", wPose);
  std::printf("           flip 下の IK と基準の差 %.2e deg\n", wRound * kDeg);
  check(wPose < 1e-12, "flip が幾何を変えてしまっている");
  check(wRound < 1e-9, "flip 下で IK が別の根に落ちる");
}

void checkUnreachable(const LegParams & prm)
{
  const double reach = prm.l3e + prm.l4 + prm.l5;
  struct Case { const char * label; Vec3 p; IkStatus want; };
  const Vec3 hip = prm.p0;
  // 「近すぎる」は (B-7) を通してから A <= 0 で落ちる形にしたいので、
  // ρ >= |a| になるよう膝軸方向に a だけ寄せておく
  const Case cases[] = {
    {"遠すぎる", hip + Vec3{0, 0, -(reach + prm.l6 + 50.0)}, IkStatus::KneeOutOfRange},
    {"近すぎる", hip + Vec3{0, prm.a, -(prm.l6 + 1.0)}, IkStatus::NoBranch},
    // 足首を膝軸方向のオフセット a より股に近づける（ρ < |a|）と (B-7) に解が無い
    {"膝軸方向に近い", hip + prm.p6 + Vec3{100.0, 0.5 * prm.a, 0}, IkStatus::AnkleOutOfRange},
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
  const Vec3 hip = right.p0;
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
  // J6 足首ロール）が反転し、Ry まわり（J1 股ピッチ, J4 膝, J5 足首ピッチ）はそのまま。
  // 幾何だけを見たいので AXIS_FLIP は落としておく。
  LegParams r2 = right, l2 = left;
  for (std::size_t k = 0; k < kNumJoints; ++k) {r2.sign[k] = 1.0; l2.sign[k] = 1.0;}
  std::mt19937_64 rng(seed);
  double wp = 0.0, wR = 0.0;
  for (int i = 0; i < n; ++i) {
    double thr[kNumJoints];
    randomTheta(r2, rng, thr);
    const double thl[kNumJoints] = {thr[0], -thr[1], -thr[2], thr[3], thr[4], -thr[5]};
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
  std::printf("片脚 FK / IK 自己検算  (C++ / Σ_B・膝と足首ピッチが同軸の導出 (B-n))\n");
  std::printf("======================================================================\n");

  for (const auto side : {Side::RIGHT, Side::LEFT}) {
    LegParams prm = makeLegParams(side);
    check(prm.valid(), "パラメータが前提を満たしていない");
    std::printf("\n[%s] ℓ3=%g ℓ4=%g ℓ5=%g ℓ6=%g  a3=%g a4=%g (a=%g) b=%g σ=%+d [Σ_B]\n",
      side == Side::RIGHT ? "right" : "left",
      prm.l3, prm.l4, prm.l5, prm.l6, prm.a3, prm.a4, prm.a, prm.b, prm.sigma);
    const Vec3 hipB = prm.p0;
    std::printf("       p0=(%g, %g, %g) [Σ_B]  flip={%d,%d,%d,%d,%d,%d}\n",
      hipB.x, hipB.y, hipB.z,
      prm.sign[0] < 0, prm.sign[1] < 0, prm.sign[2] < 0,
      prm.sign[3] < 0, prm.sign[4] < 0, prm.sign[5] < 0);
    checkZeroPose(prm);
    checkRoundtrip(prm, n, seed, "");
  }

  {
    std::printf("\n[膝軸方向・前後オフセットあり  a3=11, a4=-4, b=7]\n");
    LegParams prm = makeLegParams(Side::RIGHT);
    prm.a3 = 11.0; prm.a4 = -4.0; prm.b = 7.0; prm.finalize();
    checkZeroPose(prm);
    checkRoundtrip(prm, n, seed + 1, "");

    std::printf("\n[文書・導出の閉形式との照合]\n");
    checkClosedForms(prm, std::min(n, 2000), seed + 2);
  }

  std::printf("\n[膝軸方向オフセットの分け方の不変性 (導出 [1])]\n");
  checkXSplitInvariance(std::min(n, 2000), seed + 3);

  std::printf("\n[a = 0 / ℓ5 = 0 で簡単な形に戻ること]\n");
  checkReduceToSimple(std::min(n, 2000), seed + 4);

  {
    std::printf("\n[膝の分岐 σ = -1 (オフセットあり)]\n");
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
