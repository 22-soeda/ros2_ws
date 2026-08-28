// キーフレームモーション（攻撃・旋回・起き上がり・ホームポジション）の枠組み。
//
// ===========================================================================
// なぜ「足裏の (p, R) + 腕の生角度」なのか
// ===========================================================================
// 脚は運動学が入っているので、関節角で書くと機体を組み替えたときに全モーションが
// 死ぬ。足裏の位置姿勢で書いておけば、寸法が変わっても IK が吸収する。
// 逆に腕（ID7-10）は運動学を持たないので、サーボ角をそのまま書く以外にない。
// 両方を 1 つのキーフレームに同居させるのがこの型。
//
// ===========================================================================
// config の書式 (config/motions.yaml)
// ===========================================================================
//   単位: 位置 mm / 角度 deg / 時間 s。座標系 Σ_B（x 前・y 左・z 上、原点 = 骨盤）
//
//     motions:
//       punch_r:
//         return_home: true       # 技のあとホームへ戻る（既定 true）
//         return_time: 0.6        # 戻りにかける時間 [s]
//         keyframes:
//           - t: 0.25             # ★ひとつ前の姿勢からこの姿勢までの時間
//             R_foot: {p: [10.0, -89.3, -160.0], rpy: [0, 0, 0]}
//             L_foot: {p: [ 0.0,  89.3, -160.0], rpy: [0, 0, 0]}
//             arms:   {R8: -60.0, R9: 10.0}
//           - t: 0.15
//             arms:   {R8: 20.0}
//
// * ``t`` は絶対時刻ではなく **区間の長さ**（ユーザの言う「時間間隔」）。
//   最初のキーフレームの t は「今の姿勢からそこへ移るまでの時間」。
// * 書かなかった項目はひとつ前のキーフレームの値を引き継ぐ。上の例の 2 本目は
//   足を動かさず R8 だけ振り戻す、という意味になる。1 本目が引き継ぐ相手は
//   **再生を始めた瞬間の実際の姿勢**なので、どこから撃っても繋がる。
// * 区間の時間補間は既定で 5 次多項式（両端で速度ゼロ）。キーフレームで
//   ``ease: linear`` と書けばその区間だけ等速になる。既定を 5 次にしてあるのは、
//   等速だとキーフレームごとに速度が階段状に飛んで、二足ではそれだけで転ぶため。
//
// ティーチツール (motion_teach) が吐くのはこの書式そのままなので、脱力させた機体を
// 手で構えて捕まえ、``t:`` だけ埋めれば 1 本のモーションになる。
//
// ===========================================================================
// ホームポジションはここに書かない
// ===========================================================================
// ホーム姿勢の原本は roboone_motion/config/home_pose.yaml。こちらと同じく
// **足裏の位置姿勢**で書いてあり（高さ・前後・半間隔・rpy）、関節角は IK が決める。
// motions.yaml に "home" を書くと二重定義になるので、読み込み時に警告して捨てる。
#ifndef ROBOONE_MOTION_NODE__MOTION_LIBRARY_HPP_
#define ROBOONE_MOTION_NODE__MOTION_LIBRARY_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "roboone_motion_node/body_pose.hpp"
#include "roboone_motion_node/servo_map.hpp"

namespace roboone_motion_node
{

/// キーフレーム 1 枚。書かれなかった項目は has_* が false で、再生時に引き継ぐ。
struct KeyFrame
{
  double dt = 0.3;                  //!< [s] ひとつ前の姿勢からここへ到達するまで
  bool has_foot[kNumSide]{false, false};
  FootPose foot[kNumSide];
  std::vector<uint8_t> has_arm;     //!< ServoMap::arms() と同順・同数
  std::vector<double> arm;          //!< [deg]
  bool linear = false;              //!< ease: linear（既定は 5 次）
};

struct Motion
{
  std::string name;
  std::vector<KeyFrame> frames;
  bool return_home = true;          //!< 技のあとホームポジションへ戻る
  double return_time = 0.6;         //!< [s] 戻りにかける時間
};

class MotionLibrary
{
public:
  /// motions.yaml を読む。arms の名前は ServoMap の軸名（"R8" 等）で照合する。
  bool load(const std::string & path, const ServoMap & map, std::string & err);

  /// 技名で引く。無ければ nullptr。
  const Motion * find(const std::string & name) const;

  /// 読み込んだ技名を並べた 1 行（起動ログ用）。
  const std::string & summary() const {return summary_;}

  /// 読み込み時に出た警告（未知の腕軸名など）。空なら問題なし。
  const std::vector<std::string> & warnings() const {return warnings_;}

private:
  std::vector<Motion> motions_;
  std::string summary_;
  std::vector<std::string> warnings_;
};

/// モーション 1 本の再生。時刻を渡すと、その瞬間の姿勢を返す。
///
/// 「書かなかった項目は引き継ぐ」の解決は start() でまとめてやる。再生を始めた
/// 瞬間の実際の姿勢が起点なので、キーフレーム側では解決できない。
class MotionPlayer
{
public:
  /// from  再生開始時点の姿勢（引き継ぎの起点）
  /// home  return_home が立っているときの戻り先
  void start(const Motion & m, const BodyPose & from, const BodyPose & home, double now);

  bool active() const {return active_;}
  const std::string & name() const {return name_;}

  /// now 時点の姿勢を out に書く。再生が終わっていたら false（out は最終姿勢）。
  bool sample(double now, BodyPose & out);

  void stop() {active_ = false;}

  /// 再生の総時間 [s]。
  double duration() const {return t_.empty() ? 0.0 : t_.back();}

private:
  std::vector<BodyPose> pose_;      //!< pose_[0] = 起点、以降キーフレームごと
  std::vector<double> t_;           //!< pose_ と同順の累積時刻（t_[0] = 0）
  std::vector<uint8_t> linear_;     //!< 区間 k (pose_[k] -> pose_[k+1]) が等速か
  std::string name_;
  double t0_ = 0.0;
  bool active_ = false;
};

/// 2 つの姿勢を混ぜる。u = 0 で a、u = 1 で b。
///
/// 姿勢は RPY を成分ごとに線形補間する。回転行列の球面補間ではないので、大きく
/// 回す区間では中間の足の向きが厳密な最短経路からずれるが、足裏の傾きは
/// ankleClampJoints() の窓（±35 deg 弱）に収まる範囲でしか使えないので、
/// この範囲では差が出ない。角度を跨ぐ心配が要らないぶん扱いやすい方を採る。
BodyPose blendPose(const BodyPose & a, const BodyPose & b, double u, bool linear);

}  // namespace roboone_motion_node

#endif  // ROBOONE_MOTION_NODE__MOTION_LIBRARY_HPP_
