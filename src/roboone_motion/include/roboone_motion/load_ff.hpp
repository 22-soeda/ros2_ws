// 荷重の前送り — 脚が荷重で縮む分を、あらかじめ伸ばして返す。
//
// **ROS を知らない。** 入力は歩行計画の出力（ZMP と両足の位置）と、実測から決めた
// 沈みの量だけ。出力は足先目標の z を脚ごとにずらす量。motion_selftest から実機なしで
// 符号まで確かめられる。
//
// ===========================================================================
// 何のためのものか
// ===========================================================================
// 両足が床に着いている間、骨盤と床で閉じたループができる。このとき
// **2 本の脚の「指令上の長さの差」が、そのまま床を押し合う力になる。**
// 荷重の配分は不静定で、位置指令だけでは決められない。
//
// 実機で測った沈みは全荷重で 3〜4mm（2026-09-18 の bag。骨盤から見た支持足が
// 指令より 2.2〜4.2mm 高い = 脚がそれだけ縮んでいる）。脚のばね定数は「体重 ÷ 3.5mm」
// なので、**脚長の指令が 1mm ずれると体重の 2〜3 割が片方へ偏る。**
// 遊脚側の押しが体重の約 1/4 を超えると倒れ始める（支持足の中心から遊脚まで 140mm、
// 足裏の半幅 37mm）ので、1mm の誤差で倒れる領域に入る。
//
// 計画は両足とも床（z = 0）を指令していて、荷重で縮む分を見ていない。だから荷重が
// 移るほど、軽くなる側の脚が「長すぎる」まま取り残されて床を押す。着地の瞬間も、
// 両足支持で重心を移している最中も、起きていることは同じ。
//
// ===========================================================================
// やること
// ===========================================================================
// 各脚を「これから受け持つ荷重ぶん縮むはずの量」だけ先に伸ばす。
//
//     伸ばす量 [mm] = その脚の荷重の割合 α × sink
//
// α は計画の ZMP を 2 足を結ぶ線へ射影して出す（それが荷重配分の定義そのもの）。
// 片足支持では支持脚が 1・遊脚が 0 になるので、**遊脚には何も足さない。**
//
// この足し方は骨盤の高さについても辻褄が合う。α の合計は常に 1 なので、
//
//   * 両足で立つ  各脚 α = 0.5 → 各脚を sink/2 伸ばす → 各脚が sink/2 縮む → 骨盤は計画どおり
//   * 片足で立つ  支持脚 α = 1 → 支持脚を sink 伸ばす → sink 縮む       → 骨盤は計画どおり
//
// つまり**各脚を、その脚がこれから縮む分だけ伸ばしている**だけで、余計な上下動は入らない。
//
// ===========================================================================
// 効かせ方と限界
// ===========================================================================
//   * `sink` の既定は 0（＝今までと完全に同じ動き）。実機で測った値を入れて使う。
//   * 出す量は rate [mm/s] で頭を抑える（動歩行では割合が歩の境界で飛ぶため）。
//   * これは**前送りだけ**で、荷重を測っていない。関節の外のたわみ（足裏のつぶれ・
//     リンクのしなり・ガタ）は脚の縮みとして一括りに sink へ押し込む形になる。
//     残りは荷重フィードバック（段階 b）で閉じる。
//   * 起動時の到達域の門（checkWalkEnvelope / checkStaticWalkEnvelope）は
//     **この伸ばしを含めずに**計画の幾何だけを見る。伸ばした足先は
//     MotionController の 10Hz の見張りが機構の到達域で見る。
#ifndef ROBOONE_MOTION__LOAD_FF_HPP_
#define ROBOONE_MOTION__LOAD_FF_HPP_

#include <algorithm>

#include "roboone_motion/body_pose.hpp"
#include "roboone_motion/side.hpp"
#include "roboone_walk_core/walk_engine.hpp"

namespace roboone_motion
{

namespace rwc = roboone_walk_core;

/// 実行中に変えられる。**既定は 0**（入れても今までと同じ動き）。
struct LoadFfParams
{
  //! [mm] 全荷重（体重まるごと）が乗ったときに脚が縮む量。実機で測って入れる
  double sink = 8.0;
  //! [mm] 1 脚あたりの伸ばし量の上限（測り違い・α の暴れに対する保険）
  double clamp = 10.0;
  //! [mm/s] 伸ばし量の変化の速さの上限。**動歩行にはこれが要る**（下の注記）
  double rate = 10.0;

  bool active() const {return sink != 0.0;}
};

/// [m] 計画の足がこれ以下なら接地とみなす。静歩行の遊脚は着地でちょうど 0 になり、
/// 動歩行は td_overdrive ぶん負へ入る。浮動小数の丸めで取りこぼさない程度の許容。
constexpr double kLoadContactEps = 1e-6;

/// 各脚が受け持つ荷重の割合 [0,1]。合計 1（両足とも浮いていれば両方 0）。
///
/// 両足接地のときは、ZMP を 2 足を結ぶ線へ射影した位置がそのまま割合になる
/// （2 点で支えるときの力の配分の定義）。線の外へ出る ZMP は端で止める。
inline void loadShare(const rwc::WalkOutputs & o, double out[kNumSide])
{
  const rwc::Vec3 * foot[kNumSide] = {&o.right_foot, &o.left_foot};
  bool down[kNumSide]{false, false};
  int n = 0;
  for (int s = 0; s < kNumSide; ++s) {
    down[s] = (*foot[s])[2] <= kLoadContactEps;
    n += down[s] ? 1 : 0;
    out[s] = 0.0;
  }
  if (n == 0) {return;}                     // 両足とも浮いている（どちらの計画にも無い）
  if (n == 1) {
    out[down[kRight] ? kRight : kLeft] = 1.0;
    return;
  }
  const double dx = o.left_foot[0] - o.right_foot[0];
  const double dy = o.left_foot[1] - o.right_foot[1];
  const double d2 = dx * dx + dy * dy;
  if (d2 <= 1e-12) {                        // 足が重なっている（起動時の門が弾く）
    out[kRight] = out[kLeft] = 0.5;
    return;
  }
  const double t =
    ((o.zmp[0] - o.right_foot[0]) * dx + (o.zmp[1] - o.right_foot[1]) * dy) / d2;
  out[kLeft] = std::min(std::max(t, 0.0), 1.0);
  out[kRight] = 1.0 - out[kLeft];
}

/// 荷重の割合ぶん脚を伸ばす（足先を骨盤から遠ざける = Σ_U の z を下げる）。
///
/// **レート制限のために状態を持つ。** 割合そのものは計画から決まるが、その割合は
/// いつもなめらかとは限らない:
///
///   * 静歩行 両足支持の間に ZMP が足から足へ連続に動くので、割合もなめらか
///   * 動歩行 ZMP は歩の間ずっと支持足に固定で、歩の境界で反対の足へ**飛ぶ**
///            （ds_time = 0 のとき）。そのまま足先に出すと 1 周期で 2·sink の
///            段差になり、直そうとしている押し込みを自分で作ってしまう
///
/// なので目標へは rate [mm/s] で近づける。実際に荷重が移るのにも時間がかかるので、
/// 追い付かないこと自体は害ではない（前送りが薄まるだけ）。
/// sink を実行中に 0 へ戻したときも、跳ねずに抜けていく。
class LoadFf
{
public:
  void reset()
  {
    for (int s = 0; s < kNumSide; ++s) {off_[s] = 0.0;}
  }

  /// share には割合を必ず入れて返す（sink = 0 でも記録に出せるように）。
  void update(
    const rwc::WalkOutputs & o, const LoadFfParams & p, double dt, FootPose f[kNumSide],
    double share[kNumSide])
  {
    loadShare(o, share);
    const double lim = std::max(0.0, p.clamp);
    const double step = std::max(0.0, p.rate) * std::max(0.0, dt);
    for (int s = 0; s < kNumSide; ++s) {
      const double tgt =
        p.active() ? std::min(std::max(share[s] * p.sink, -lim), lim) : 0.0;
      const double d = tgt - off_[s];
      off_[s] += std::min(std::max(d, -step), step);
      f[s].p.z -= off_[s];
    }
  }

  /// [mm] 今出している伸ばし量（記録用）。
  const double * offset() const {return off_;}

private:
  double off_[kNumSide]{0.0, 0.0};
};

}  // namespace roboone_motion

#endif  // ROBOONE_MOTION__LOAD_FF_HPP_
