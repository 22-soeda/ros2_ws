// 膝 4 節リンク 順変換 / 逆変換の自己検算。
//   ros2 run roboone_kinematics knee_selftest [-n 姿勢数] [--seed S]
//
// scripts/test_knee_fourbar.py の受け入れ基準 1〜8 をそのまま C++ で走らせ、
// docs/膝4節リンク導出.pdf §9 の数値検算と参照値を再現する。閾値は Python 側と
// 同じで、緩めていない。
//
//   1. FK の出力でループが閉じる                       残差 < 1e-9 mm
//   2. IK(FK(θ2)) == θ2                               誤差 < 1e-9 deg
//   3. 速度が数値微分と一致                            誤差 < 1e-6
//   4. 加速度が数値微分と一致（複数姿勢）              誤差 < 1e-4
//   5. 伝達比の解析式が数値微分と一致                  誤差 < 1e-6
//   6. r4 = 26 mm でも 1〜5 が通る（r1 = r4 の簡約が無いことの検査）
//   7. 退化ケース（平行四辺形・r1 = 0）
//   8. 到達不能で status が返る（黙って NaN を返さない）
//
// さらに実機の繋ぎこみとして
//   9. σ_knee が θ4_zero と枝から一意に決まること
//  10. 脚 IK の公開角 θ4 <-> 曲げ量 <-> サーボ指令 の往復と、
//      曲げ量 0 で脚が本当に伸び切る（T ポーズ）こと
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

#include "roboone_kinematics/leg_servo.hpp"

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

constexpr double kDeg = M_PI / 180.0;
double deg(double r) {return r * 180.0 / M_PI;}
double wrapPi(double a) {return std::atan2(std::sin(a), std::cos(a));}
//! 数値微分の刻み。基準 3〜5 は Python 側と同じ h を使う
constexpr double kH = 1e-4;

//! θ2 = 160°〜270° を 0.5° 刻み（221 点）
std::vector<double> sweep()
{
  std::vector<double> v;
  for (int i = 0; i <= 220; ++i) {v.push_back((160.0 + 0.5 * i) * kDeg);}
  return v;
}

//! 加速度は姿勢で式の壊れ方が変わる。230° 付近は誤った式でも偶然近い値が出るので
//! 1 点だけの一致で判断しない。
const double kPoses[] = {170.0, 185.0, 200.0, 230.0, 245.0, 260.0};

// ---------------------------------------------------------------------------
// 基準 1・2  ループと往復
// ---------------------------------------------------------------------------
void testLoopAndRoundTrip(const KneeParams & prm, const char * tag)
{
  double wLoop = 0.0, wRt = 0.0, wRt4 = 0.0;
  for (double t2 : sweep()) {
    KneePose p;
    check(kneeFk(prm, t2, p) == KneeStatus::Ok, "FK が解けない");
    // A + r3·e(θ3) − B
    const double rx = p.a.x + prm.r3 * std::cos(p.theta3) - p.b.x;
    const double ry = p.a.y + prm.r3 * std::sin(p.theta3) - p.b.y;
    wLoop = std::max(wLoop, std::hypot(rx, ry));
    wLoop = std::max(wLoop, std::fabs(p.coupler().norm() - prm.r3));

    KneePose q;
    check(kneeIk(prm, p.theta4, q) == KneeStatus::Ok, "IK が解けない");
    wRt = std::max(wRt, std::fabs(deg(wrapPi(q.theta2 - t2))));

    KneePose r;
    check(kneeFk(prm, q.theta2, r) == KneeStatus::Ok, "FK(IK) が解けない");
    wRt4 = std::max(wRt4, std::fabs(deg(wrapPi(r.theta4 - p.theta4))));
  }
  std::printf("  [%s] ループ残差 %.2e mm / IK(FK) %.2e deg / FK(IK) %.2e deg\n",
    tag, wLoop, wRt, wRt4);
  check(wLoop < 1e-9, "基準 1: ループが閉じない");
  check(wRt < 1e-9, "基準 2: 往復しない（枝の取り違えを疑う）");
  check(wRt4 < 1e-9, "基準 2: θ4 側の往復が閉じない");
}

//! 枝を逆に取ると往復が大きくずれること（基準 2 が効いていることの確認）。
//! 既存 MATLAB 実装は β の符号が逆で、最大 102 度ずれる。
void testWrongBranch(const KneeParams & base)
{
  KneeParams bad = base;
  bad.beta = -base.beta;
  double worst = 0.0;
  for (double t2 : sweep()) {
    KneePose p, q;
    if (kneeFk(bad, t2, p) != KneeStatus::Ok) {continue;}
    if (kneeIk(bad, p.theta4, q) != KneeStatus::Ok) {continue;}
    worst = std::max(worst, std::fabs(deg(wrapPi(q.theta2 - t2))));
  }
  std::printf("  枝を逆に取ったときの往復誤差 最大 %.1f deg（既存 MATLAB 実装のずれと同じ）\n",
    worst);
  check(worst > 10.0, "枝を逆にしても往復してしまう＝基準 2 が効いていない");
}

// ---------------------------------------------------------------------------
// 基準 3・4・5  速度・加速度・伝達比
// ---------------------------------------------------------------------------
void testVelocityAccelRatio(const KneeParams & prm, const char * tag)
{
  double wV = 0.0, wA = 0.0, wR = 0.0;
  for (double d2 : kPoses) {
    const double t2 = d2 * kDeg;
    KneePose p, pp, pm;
    check(kneeFk(prm, t2, p) == KneeStatus::Ok, "FK が解けない");
    check(kneeFk(prm, t2 + kH, pp) == KneeStatus::Ok, "FK(+h) が解けない");
    check(kneeFk(prm, t2 - kH, pm) == KneeStatus::Ok, "FK(-h) が解けない");

    // 基準 3: ω2 = 1 での ω3, ω4
    double w3 = 0.0, w4 = 0.0;
    check(kneeVelocity(prm, p, 1.0, w3, w4) == KneeStatus::Ok, "速度が解けない");
    wV = std::max(wV, std::fabs(w3 - wrapPi(pp.theta3 - pm.theta3) / (2.0 * kH)));
    wV = std::max(wV, std::fabs(w4 - wrapPi(pp.theta4 - pm.theta4) / (2.0 * kH)));

    // 基準 4: ω2 = α2 = 1。θ2(t) = θ2 + t + t²/2 を入れて 2 階微分する
    double a3 = 0.0, a4 = 0.0;
    check(kneeAcceleration(prm, p, 1.0, 1.0, a3, a4) == KneeStatus::Ok, "加速度が解けない");
    KneePose qp, qm;
    check(kneeFk(prm, t2 + kH + 0.5 * kH * kH, qp) == KneeStatus::Ok, "FK(+) が解けない");
    check(kneeFk(prm, t2 - kH + 0.5 * kH * kH, qm) == KneeStatus::Ok, "FK(-) が解けない");
    const double n3 = (wrapPi(qp.theta3 - p.theta3) + wrapPi(qm.theta3 - p.theta3)) / (kH * kH);
    const double n4 = (wrapPi(qp.theta4 - p.theta4) + wrapPi(qm.theta4 - p.theta4)) / (kH * kH);
    wA = std::max(wA, std::max(std::fabs(a3 - n3), std::fabs(a4 - n4)));

    // 基準 5: 伝達比は FK 側の数値微分とも比べる
    double ratio = 0.0;
    check(kneeRatio(prm, p, ratio) == KneeStatus::Ok, "伝達比が解けない");
    wR = std::max(wR, std::fabs(ratio - wrapPi(pp.theta4 - pm.theta4) / (2.0 * kH)));
  }

  // 基準 5 本体: IK の数値微分（dθ2/dθ4 の逆数）と比べる。全掃引点で見る
  for (double t2 : sweep()) {
    KneePose p, ip, im;
    if (kneeFk(prm, t2, p) != KneeStatus::Ok) {continue;}
    double ratio = 0.0;
    if (kneeRatio(prm, p, ratio) != KneeStatus::Ok) {continue;}
    if (kneeIk(prm, p.theta4 + kH, ip) != KneeStatus::Ok) {continue;}
    if (kneeIk(prm, p.theta4 - kH, im) != KneeStatus::Ok) {continue;}
    wR = std::max(wR, std::fabs(ratio - 2.0 * kH / wrapPi(ip.theta2 - im.theta2)));
  }

  std::printf("  [%s] 速度 %.2e / 加速度 %.2e / 伝達比 %.2e\n", tag, wV, wA, wR);
  check(wV < 1e-6, "基準 3: 速度が数値微分と合わない");
  check(wA < 1e-4, "基準 4: 加速度が数値微分と合わない");
  check(wR < 1e-6, "基準 5: 伝達比が数値微分と合わない");
}

//! M の行と右辺を **片方だけ** 反転すると加速度だけが壊れることの確認。
//! 両方同時に反転しても速度は不変なので、基準 4 が無いと取り違えが素通りする。
void testSignPairing(const KneeParams & prm)
{
  double worstV = 0.0, worstA = 0.0;
  for (double d2 : kPoses) {
    KneePose p;
    if (kneeFk(prm, d2 * kDeg, p) != KneeStatus::Ok) {continue;}
    double w3 = 0.0, w4 = 0.0, a3 = 0.0, a4 = 0.0;
    kneeVelocity(prm, p, 1.0, w3, w4);
    kneeAcceleration(prm, p, 1.0, 1.0, a3, a4);

    const kdetail::Mat2 m = kdetail::loopMatrix(prm, p);
    const double s2 = std::sin(p.theta2), c2 = std::cos(p.theta2);
    const double s3 = std::sin(p.theta3), c3 = std::cos(p.theta3);
    const double s4 = std::sin(p.theta4), c4 = std::cos(p.theta4);
    // 右辺だけ符号を反転した「壊れた」加速度
    const double b1 = -(prm.r2 * s2 + prm.r2 * c2 + prm.r3 * w3 * w3 * c3 -
      prm.r4 * w4 * w4 * c4);
    const double b2 = -(-prm.r2 * c2 + prm.r2 * s2 + prm.r3 * w3 * w3 * s3 -
      prm.r4 * w4 * w4 * s4);
    const double bad4 = (m.m11 * b2 - b1 * m.m21) / m.det;
    worstA = std::max(worstA, std::fabs(bad4 - a4));
    // 速度は M と右辺を揃えて反転しても不変
    const double v4 = (m.m11 * (prm.r2 * c2) - (-prm.r2 * s2) * m.m21) / m.det;
    worstV = std::max(worstV, std::fabs(-v4 - w4));
  }
  std::printf("  符号の対: 同時反転で速度は不変 %.2e / 片側だけ反転で加速度が壊れる %.2f\n",
    worstV, worstA);
  check(worstV < 1e-12, "速度は符号の同時反転で不変のはず");
  check(worstA > 1.0, "右辺だけ反転しても加速度が変わらない＝基準 4 が効いていない");
}

// ---------------------------------------------------------------------------
// 基準 6  r1 = r4 に暗黙依存した簡約が無いこと
// ---------------------------------------------------------------------------
void testGenericDimensions()
{
  KneeParams prm = makeKneeParams(Side::RIGHT);
  prm.r4 = 26.0;
  double wLen = 0.0, wAng = 0.0;
  for (double t2 : sweep()) {
    KneePose p;
    if (kneeFk(prm, t2, p) != KneeStatus::Ok) {continue;}
    const double wn = std::hypot(p.b.x - prm.r1, p.b.y);
    const double naive = std::sqrt(
      std::max(0.0, 2.0 * prm.r1 * prm.r1 * (1.0 - std::cos(p.theta4))));
    wLen = std::max(wLen, std::fabs(wn - naive));
    wAng = std::max(wAng, std::fabs(deg(wrapPi(
      std::atan2(p.b.y, p.b.x - prm.r1) - (p.theta4 + M_PI) / 2.0))));
  }
  std::printf("  r4 = 26 で「r1 = r4 の簡約」とのずれ: 長さ %.2f mm / 角度 %.2f deg\n",
    wLen, wAng);
  check(wLen > 5.0, "基準 6: 簡約とのずれが小さすぎる（簡約が入っている疑い）");
  check(wAng > 3.0, "基準 6: 角のずれが小さすぎる");

  for (double r4 : {14.0, 20.0, 26.0, 33.0}) {
    KneeParams q = makeKneeParams(Side::RIGHT);
    q.r4 = r4;
    double wRt = 0.0;
    for (double t2 : sweep()) {
      KneePose p, b;
      if (kneeFk(q, t2, p) != KneeStatus::Ok) {continue;}
      if (kneeIk(q, p.theta4, b) != KneeStatus::Ok) {continue;}
      wRt = std::max(wRt, std::fabs(deg(wrapPi(b.theta2 - t2))));
    }
    check(wRt < 1e-9, "基準 6: r4 を振ると往復しない");
  }
  std::printf("  r4 = 14 / 20 / 26 / 33 mm でも往復は 1e-9 deg 以内\n");
}

// ---------------------------------------------------------------------------
// 基準 7  退化ケース
// ---------------------------------------------------------------------------
void testDegenerateCases()
{
  // 平行四辺形（r2 = r4 かつ r3 = r1）は θ4 = θ2 の恒等写像。
  // ただし θ2 = 0°, 180° で 4 本が一直線に並ぶ「変化点」があり、そこで 2 交点が
  // 重なって枝が入れ替わる。変化点をまたがない範囲では枝は定数のまま。
  double worst = 0.0;
  const int branchBeta[2] = {+1, -1};
  const int branchEps[2] = {-1, +1};
  const double from[2] = {2.0, 182.0};
  for (int k = 0; k < 2; ++k) {
    KneeParams prm;
    prm.r1 = 20.0; prm.r2 = 45.0; prm.r3 = 20.0; prm.r4 = 45.0;
    prm.beta = branchBeta[k];
    prm.eps = branchEps[k];
    for (int i = 0; i < 89; ++i) {
      const double t2 = (from[k] + 2.0 * i) * kDeg;
      KneePose p, q;
      check(kneeFk(prm, t2, p) == KneeStatus::Ok, "平行四辺形で FK が解けない");
      check(kneeIk(prm, t2, q) == KneeStatus::Ok, "平行四辺形で IK が解けない");
      worst = std::max(worst, std::fabs(deg(wrapPi(p.theta4 - t2))));
      worst = std::max(worst, std::fabs(deg(wrapPi(q.theta2 - t2))));
    }
    // 恒等写像なら伝達比は 1
    KneePose p;
    kneeFk(prm, (from[k] + 40.0) * kDeg, p);
    double ratio = 0.0;
    kneeRatio(prm, p, ratio);
    check(std::fabs(ratio - 1.0) < 1e-9, "平行四辺形で伝達比が 1 でない");
    // 実測 1 姿勢から同じ枝が復元される
    int b = 0, e = 0;
    check(kneeBranchesFromPose(prm, p.theta2, p.theta4, b, e) == KneeStatus::Ok,
      "枝が復元できない");
    check(b == prm.beta && e == prm.eps, "復元した枝が組み方と合わない");
  }
  std::printf("  平行四辺形: max|θ4 − θ2| = %.2e deg（変化点の両側それぞれで恒等写像）\n", worst);
  check(worst < 1e-9, "基準 7: 平行四辺形が恒等写像にならない");

  // r1 = 0（モータ軸が膝軸と同軸）は θ2 − θ4 が一定。
  KneeParams co;
  co.r1 = 0.0; co.r2 = 45.0; co.r3 = 35.0; co.r4 = 20.0;
  double lo = 1e9, hi = -1e9;
  for (int i = 0; i < 120; ++i) {
    KneePose p;
    const double t2 = 3.0 * i * kDeg;
    check(kneeFk(co, t2, p) == KneeStatus::Ok, "r1 = 0 で FK が解けない");
    const double d = deg(wrapPi(p.theta4 - t2));
    lo = std::min(lo, d); hi = std::max(hi, d);
  }
  const double want = deg(std::acos(
      (co.r2 * co.r2 + co.r4 * co.r4 - co.r3 * co.r3) / (2.0 * co.r2 * co.r4)));
  std::printf("  r1 = 0: θ2 − θ4 のばらつき %.2e deg（一定値 %.4f deg）\n", hi - lo, std::fabs(lo));
  check(hi - lo < 1e-9, "基準 7: r1 = 0 で θ2 − θ4 が一定でない");
  check(std::fabs(std::fabs(lo) - want) < 1e-9, "基準 7: オフセットが余弦定理と合わない");
}

// ---------------------------------------------------------------------------
// 基準 8  到達不能の扱い
// ---------------------------------------------------------------------------
void testUnreachable(const KneeParams & prm)
{
  KneePose p;
  check(!kneeAssemblableTheta2(prm, 0.0), "θ2 = 0° が組めることになっている");
  check(kneeFk(prm, 0.0, p) == KneeStatus::Unreachable, "基準 8: θ2 = 0° で Unreachable が返らない");

  // 組めない θ2 で NaN が返らないこと
  for (double d2 : {0.0, 30.0, 60.0, 70.0, 300.0, 350.0}) {
    KneePose q;
    if (kneeFk(prm, d2 * kDeg, q) == KneeStatus::Ok) {
      check(std::isfinite(q.theta3) && std::isfinite(q.theta4), "基準 8: NaN が返った");
      check(kneeAssemblableTheta2(prm, d2 * kDeg), "組めない θ2 で Ok が返った");
    }
  }

  // 死点では伝達比・速度が DeadPoint
  const double c = ((prm.r3 + prm.r4) * (prm.r3 + prm.r4) - prm.r1 * prm.r1 -
    prm.r2 * prm.r2) / (2.0 * prm.r1 * prm.r2);
  KneePose dead;
  check(kneeFk(prm, std::acos(detail::clampUnit(c)), dead) == KneeStatus::Ok,
    "死点の姿勢が作れない");
  double ratio = 0.0, w3 = 0.0, w4 = 0.0;
  check(kneeRatio(prm, dead, ratio) == KneeStatus::DeadPoint, "基準 8: 死点で DeadPoint が返らない");
  check(kneeVelocity(prm, dead, 1.0, w3, w4) == KneeStatus::DeadPoint,
    "基準 8: 死点で速度が DeadPoint を返さない");
  std::printf("  到達不能・死点はすべて status で返り、NaN は返らない\n");
}

// ---------------------------------------------------------------------------
// 9  σ_knee の一意性
// ---------------------------------------------------------------------------
// θ4_zero（T ポーズでのロッカー角）と枝が決まると、σ_knee は選べない。
// σ = -1 は設計可動域の途中で死点に当たる。
void testSigmaUniqueness(const KneeParams & prm)
{
  for (int sig : {+1, -1}) {
    KneeParams q = prm;
    q.sigmaJoint = sig;
    double reached = 0.0, ratioMin = 1e9;
    for (int i = 0; i <= 1500; ++i) {
      const double bend = (i / 10.0) * kDeg;
      KneePose p;
      if (kneeIk(q, kneeRockerFromBend(q, bend), p) != KneeStatus::Ok) {break;}
      double r = 0.0;
      if (kneeRatio(q, p, r) != KneeStatus::Ok) {break;}
      ratioMin = std::min(ratioMin, std::fabs(r));
      reached = i / 10.0;
    }
    std::printf("  σ = %+d: 屈曲 0〜%.1f° まで到達（伝達比の最小 %.3f）\n", sig, reached, ratioMin);
    if (sig == prm.sigmaJoint) {
      check(reached >= 150.0, "設計可動域 0〜150° に届かない");
      check(ratioMin > 1.0, "作動域で伝達比が 1 を割っている");
    } else {
      check(reached < 150.0, "9: σ の一意性が崩れている（両方の向きで届いてしまう）");
    }
  }
}

// ---------------------------------------------------------------------------
// 10  脚 IK との繋ぎこみ
// ---------------------------------------------------------------------------
void testLegBridge()
{
  const LegServoParams prm = makeLegServoParams(Side::RIGHT);

  // 曲げ量 0 で脚が本当に伸び切る（T ポーズ）ことを、脚 FK の関節位置で確かめる。
  // 伸び切りなら股中心 o3 から足首ロール軸 o5 までが ℓ3 + ℓ4 ちょうどになる。
  double theta[kNumJoints] = {0, 0, 0, 0, 0, 0};
  theta[KNEE] = legAngleFromKneeBend(prm.leg, 0.0);
  Vec3 o[5];
  jointOrigins(prm.leg, theta, o);
  const double straight = (o[2] - o[0]).norm();
  const double want = prm.leg.l3e + prm.leg.l4;
  std::printf("  曲げ量 0: |o3 → o5| = %.6f mm（伸び切り ℓ3' + ℓ4 = %.6f mm）\n", straight, want);
  check(std::fabs(straight - want) < 1e-9, "10: 曲げ量 0 で脚が伸び切っていない");
  check(std::fabs(deg(theta[KNEE])) < 1e-12, "10: 既定値では bend = θ4 のはず");

  // そのときのロッカー角が θ4_zero = 89.3°、対応するサーボ角
  double servo0 = 0.0;
  check(legKneeServoFromAngle(prm, theta[KNEE], servo0) == LegServoStatus::Ok,
    "10: T ポーズで膝サーボが解けない");
  std::printf("  T ポーズ: θ4_rocker = %.4f° / θ2 = %.4f°\n",
    deg(kneeRockerFromBend(prm.knee, 0.0)), deg(kneeCrankFromServo(prm.knee, servo0)));
  check(std::fabs(deg(kneeRockerFromBend(prm.knee, 0.0)) - 89.3) < 1e-9,
    "10: T ポーズのロッカー角が 89.3° でない");

  // 公開角 -> サーボ -> 公開角 の往復（設計可動域 0〜150°）
  double worst = 0.0, servoLo = 1e9, servoHi = -1e9;
  for (int i = 0; i <= 300; ++i) {
    const double bend = (i * 0.5) * kDeg;
    const double th4 = legAngleFromKneeBend(prm.leg, bend);
    double servo = 0.0, back = 0.0;
    check(legKneeServoFromAngle(prm, th4, servo) == LegServoStatus::Ok, "10: 指令側が解けない");
    check(legKneeAngleFromServo(prm, servo, back) == LegServoStatus::Ok, "10: 観測側が解けない");
    worst = std::max(worst, std::fabs(deg(wrapPi(back - th4))));
    servoLo = std::min(servoLo, servo);
    servoHi = std::max(servoHi, servo);
  }
  std::printf("  θ4 -> サーボ -> θ4 の往復誤差 %.2e deg（屈曲 0〜150°）\n", worst);
  std::printf("  必要なサーボ振り幅 %.2f°（%.2f° 〜 %.2f°）\n",
    deg(servoHi - servoLo), deg(servoLo), deg(servoHi));
  check(worst < 1e-9, "10: 関節角 <-> サーボ指令が往復しない");

  // 脚 IK 全体との繋ぎこみ: ランダム姿勢で fk -> ik -> サーボ -> 関節角
  std::mt19937_64 rng(20260828);
  double worstAll = 0.0;
  int n = 0;
  for (int i = 0; i < 2000; ++i) {
    double th[kNumJoints];
    for (std::size_t k = 0; k < kNumJoints; ++k) {
      std::uniform_real_distribution<double> d(
        config::JOINT_LIMIT_LO_DEG[k] * kDeg, config::JOINT_LIMIT_HI_DEG[k] * kDeg);
      th[k] = d(rng);
    }
    th[KNEE] = legAngleFromKneeBend(prm.leg, std::fabs(th[KNEE]));

    Vec3 p; Mat3 R;
    fk(prm.leg, th, p, R);
    double back[kNumJoints] = {0, 0, 0, 0, 0, 0};
    if (ik(prm.leg, p, R, back, /*clamp=*/false) != IkStatus::Ok) {continue;}

    double servo = 0.0, again = 0.0;
    if (legKneeServoFromAngle(prm, back[KNEE], servo) != LegServoStatus::Ok) {continue;}
    if (legKneeAngleFromServo(prm, servo, again) != LegServoStatus::Ok) {continue;}
    worstAll = std::max(worstAll, std::fabs(deg(wrapPi(again - th[KNEE]))));
    ++n;
  }
  std::printf("  脚 IK -> 膝サーボ -> 膝関節角 の往復 %d 姿勢: 最大 %.2e deg\n", n, worstAll);
  check(n > 1500, "10: 可動域内なのに解けない姿勢が多すぎる");
  check(worstAll < 1e-9, "10: 脚 IK と繋ぐと往復しない");
}

// ---------------------------------------------------------------------------
// 参照値（依頼 §5 / 文書 §9）
// ---------------------------------------------------------------------------
void testReferenceValues(const KneeParams & prm)
{
  struct Row {double t2, t3, t4, gamma;};
  const Row rows[] = {
    {180.0, 34.05, 78.46, 44.42}, {200.0, 69.34, 119.79, 50.45},
    {220.0, 97.69, 163.26, 65.58}, {240.0, 117.05, 202.95, 85.90},
    {260.0, 129.45, 239.82, 110.38}};
  std::printf("     θ2      θ3       θ4    膝角     γ    dθ4/dθ2\n");
  for (const Row & r : rows) {
    KneePose p;
    check(kneeFk(prm, r.t2 * kDeg, p) == KneeStatus::Ok, "代表点で FK が解けない");
    double ratio = 0.0;
    kneeRatio(prm, p, ratio);
    std::printf("  %6.1f %8.2f %8.2f %7.2f %7.2f %7.3f\n",
      r.t2, deg(p.theta3), deg(p.theta4), deg(p.theta4) - 90.0,
      deg(kneeTransmissionAngle(p)), ratio);
    check(std::fabs(deg(p.theta3) - r.t3) < 0.01, "θ3 が参照値と合わない");
    check(std::fabs(deg(p.theta4) - r.t4) < 0.01, "θ4 が参照値と合わない");
    check(std::fabs(deg(kneeTransmissionAngle(p)) - r.gamma) < 0.01, "γ が参照値と合わない");
  }

  // 組める θ2 の境界は cos θ2 = 1/3、すなわち arccos(1/3) = 70.5288°
  const double edge = deg(std::acos(1.0 / 3.0));
  check(std::fabs(edge - 70.5288) < 1e-3, "境界の閉形式が合わない");
  check(kneeAssemblableTheta2(prm, (edge + 0.01) * kDeg), "境界の内側が組めない");
  check(!kneeAssemblableTheta2(prm, (edge - 0.01) * kDeg), "境界の外側が組めてしまう");

  // 設計可動域（膝角 0〜120°）の伝達比・伝達角
  double rLo = 1e9, rHi = -1e9, gLo = 1e9, gHi = -1e9, sLo = 1e9, sHi = -1e9;
  for (int i = 0; i <= 500; ++i) {
    KneePose p;
    const double bend = (120.0 * i / 500.0) * kDeg;
    check(kneeIk(prm, kneeRockerFromBend(prm, bend), p) == KneeStatus::Ok, "可動域内で解けない");
    double ratio = 0.0;
    kneeRatio(prm, p, ratio);
    rLo = std::min(rLo, ratio); rHi = std::max(rHi, ratio);
    const double g = deg(kneeTransmissionAngle(p));
    gLo = std::min(gLo, g); gHi = std::max(gHi, g);
    sLo = std::min(sLo, deg(p.theta2)); sHi = std::max(sHi, deg(p.theta2));
  }
  std::printf("  膝角 0〜120°: θ2 %.2f°〜%.2f°（振り幅 %.2f°）"
    " 伝達比 %.3f〜%.3f  γ %.2f°〜%.2f°\n", sLo, sHi, sHi - sLo, rLo, rHi, gLo, gHi);
  check(rHi > 2.2 && rLo > 1.8, "伝達比が参照値と合わない");
  check(gLo > 40.0 && gHi < 140.0, "伝達角が設計の目安 40〜140° を外れている");

  // Grashof と Freudenstein
  double sorted[4] = {prm.r1, prm.r2, prm.r3, prm.r4};
  std::sort(sorted, sorted + 4);
  check(sorted[0] + sorted[3] > sorted[1] + sorted[2], "非 Grashof でなくなっている");
  double worst = 0.0;
  for (double t2 : sweep()) {
    KneePose p;
    if (kneeFk(prm, t2, p) != KneeStatus::Ok) {continue;}
    worst = std::max(worst, std::fabs(kneeFreudensteinResidual(prm, p)));
  }
  std::printf("  非 Grashof（s+l = %.0f > p+q = %.0f）/ Freudenstein 残差 %.2e\n",
    sorted[0] + sorted[3], sorted[1] + sorted[2], worst);
  check(worst < 1e-12, "Freudenstein の残差が大きい");
}

// ---------------------------------------------------------------------------
void testTiming(const KneeParams & prm)
{
  const int n = 200000;
  KneePose p;
  auto t0 = std::chrono::steady_clock::now();
  double acc = 0.0;
  for (int i = 0; i < n; ++i) {
    const double bend = (i % 1500) * 0.1 * kDeg;
    if (kneeIk(prm, kneeRockerFromBend(prm, bend), p) == KneeStatus::Ok) {acc += p.theta2;}
  }
  const double us = std::chrono::duration<double, std::micro>(
    std::chrono::steady_clock::now() - t0).count() / n;
  std::printf("  逆変換 1 回あたり %.3f us（%d 回平均、acc %.3f）\n", us, n, acc);
  check(us < 5.0, "200Hz の制御ループには十分速いはずだが遅すぎる");
}

}  // namespace

int main(int argc, char ** argv)
{
  int n = 3000;
  std::uint64_t seed = 20260828;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
      n = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
      seed = std::strtoull(argv[++i], nullptr, 10);
    } else {
      std::fprintf(stderr, "usage: %s [-n 姿勢数] [--seed S]\n", argv[0]);
      return 2;
    }
  }
  (void)n; (void)seed;

  const KneeParams prm = makeKneeParams(Side::RIGHT);
  std::printf("膝 4 節リンク 自己検算（右脚）\n");
  std::printf("寸法は knee_config.hpp: r1=%g r2=%g r3=%g r4=%g [mm] β=%+d ε=%+d\n",
    prm.r1, prm.r2, prm.r3, prm.r4, prm.beta, prm.eps);
  std::printf("θ4_zero = %g°（T ポーズ・確定値） σ_knee = %+d\n\n",
    deg(prm.theta4Zero), prm.sigmaJoint);
  check(prm.valid(), "パラメータが前提を満たしていない");

  KneeParams r26 = prm;
  r26.r4 = 26.0;

  std::printf("[基準 1・2] ループと往復\n");
  testLoopAndRoundTrip(prm, "r4=20");
  testLoopAndRoundTrip(r26, "r4=26");
  testWrongBranch(prm);

  std::printf("\n[基準 3・4・5] 速度・加速度・伝達比\n");
  testVelocityAccelRatio(prm, "r4=20");
  testVelocityAccelRatio(r26, "r4=26");
  testSignPairing(prm);

  std::printf("\n[基準 6] 寸法の一般性\n");
  testGenericDimensions();

  std::printf("\n[基準 7] 退化ケース\n");
  testDegenerateCases();

  std::printf("\n[基準 8] 到達不能の扱い\n");
  testUnreachable(prm);

  std::printf("\n[9] σ_knee の一意性\n");
  testSigmaUniqueness(prm);

  std::printf("\n[10] 脚 IK との繋ぎこみ\n");
  testLegBridge();

  std::printf("\n[参照値]\n");
  testReferenceValues(prm);

  std::printf("\n[速度]\n");
  testTiming(prm);

  std::printf("\n%s（不一致 %d 件）\n", g_failures ? "*** 失敗" : "全部一致", g_failures);
  return g_failures ? 1 : 0;
}
