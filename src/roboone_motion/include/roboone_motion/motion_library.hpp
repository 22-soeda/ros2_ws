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
// 脚をサーボ角で書く（R_leg / L_leg）
// ===========================================================================
// 足裏書きは「IK で解ける姿勢しか書けない」という裏返しを持つ。可動域の縁、
// 足首の特異点の近く、寝ている姿勢——**順変換では出せるのに IK では戻せない**
// 領域は実際にあって（motion_teach がキーフレームごとに往復誤差を見ているのは
// このため）、そこを狙う技は足裏書きでは書けない。逃げ道として、脚も腕と同じ
// ようにサーボ角で書けるようにしてある。
//
//     - t: 0.20
//       R_leg: {ID1: -12.5, ID4: 41.2}     # T ポーズ基準の deg。書いた軸だけ効く
//       L_foot: {p: [...], rpy: [...]}     # 左は今までどおり IK
//
// * 角度は**腕と同じ T ポーズ基準 [deg]**（servo_home.yaml の原点が 0）。
//   motion_teach の画面の「サーボ[deg]」の列がそのままこれ。
// * 軸は ID で書く（ID1..ID4 と足首の ID6 / ID5）。**足首は鎖の角度**であって
//   足裏のピッチ・ロールではない。「つま先を 5 度下げる」は足裏書きの仕事。
// * 書いた軸だけが効く。同じキーフレームに R_foot と R_leg を両方書くと、
//   **R_foot で姿勢を決めてから R_leg の軸だけ上書きする**（「この姿勢の膝だけ
//   もう 5 度」が書ける）。
// * 角度書きの側は IK も FK も通らずにサーボへ出る。**届かない姿勢が書けてしまう**
//   ので、servo_limits.yaml の窓に収まっているかは起動時に照合する。
//
// ===========================================================================
// 書き方が混ざる区間
// ===========================================================================
// 1 本の技の中で足裏書きと角度書きが隣り合ったら、その区間は**角度空間で補間する**
// （足裏側を IK でサーボ角に直してから混ぜる）。足裏空間に寄せると、角度書きの端が
// IK を通ることになって「IK で戻せない姿勢を書く」という目的が消えるため。
// 両端とも足裏書きの区間は今までどおり (p, R) の直線補間で、既存の技の挙動は
// 1 mm も変わらない。
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
// * 脚は ``R_foot`` / ``L_foot``（IK）の代わりに ``R_leg`` / ``L_leg``（サーボ角）
//   でも書ける。上の節を参照。
//
// ティーチツール (motion_teach) が吐くのはこの書式そのままなので、脱力させた機体を
// 手で構えて捕まえ、``t:`` だけ埋めれば 1 本のモーションになる。``--format angle``
// を付ければ足裏の代わりに ``R_leg`` / ``L_leg`` の行が出る。
//
// ===========================================================================
// ホームポジションはここに書かない
// ===========================================================================
// ホーム姿勢の原本は roboone_walk_ref/config/home_pose.yaml。こちらと同じく
// **足裏の位置姿勢**で書いてあり（高さ・前後・半間隔・rpy）、関節角は IK が決める。
// motions.yaml に "home" を書くと二重定義になるので、読み込み時に警告して捨てる。
#ifndef ROBOONE_MOTION__MOTION_LIBRARY_HPP_
#define ROBOONE_MOTION__MOTION_LIBRARY_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "roboone_motion/body_pose.hpp"
#include "roboone_motion/servo_map.hpp"

namespace roboone_motion
{

/// キーフレーム 1 枚。書かれなかった項目は has_* が false で、再生時に引き継ぐ。
struct KeyFrame
{
  double dt = 0.3;                  //!< [s] ひとつ前の姿勢からここへ到達するまで
  bool has_foot[kNumSide]{false, false};
  FootPose foot[kNumSide];
  //! 脚をサーボ角で書いた軸（R_leg / L_leg）。並びは Joint enum
  uint8_t has_leg[kNumSide][rk::kNumJoints]{};
  double leg_servo[kNumSide][rk::kNumJoints]{};   //!< 絶対サーボ角 [rad]・Σ_U
  std::vector<uint8_t> has_arm;     //!< ServoMap::arms() と同順・同数
  std::vector<double> arm;          //!< [deg]
  bool linear = false;              //!< ease: linear（既定は 5 次）

  /// この枚で片脚を 1 軸でもサーボ角で書いているか。
  bool anyLegServo(int side) const
  {
    for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
      if (has_leg[side][j]) {return true;}
    }
    return false;
  }
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

  /// 読み込んだ技の全体（起動時の照合用。再生には find() を使う）。
  const std::vector<Motion> & motions() const {return motions_;}

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
  /// map   脚のパラメータ。書き方が混ざる区間のために、キーフレームごとに
  ///       足裏 <-> サーボ角 の影を作るのに使う（冒頭「書き方が混ざる区間」）
  void start(
    const Motion & m, const BodyPose & from, const BodyPose & home, double now,
    const ServoMap & map);

  /// 直前の start() で出た警告。空なら問題なし（IK で影を作れなかった枚など）。
  const std::string & warning() const {return warning_;}

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
  std::string warning_;
  double t0_ = 0.0;
  bool active_ = false;
};

/// 2 つの姿勢を混ぜる。u = 0 で a、u = 1 で b。
///
/// 脚は**片端でもサーボ角書きなら角度空間**で、両端とも足裏書きなら足裏空間で
/// 混ぜる（冒頭「書き方が混ざる区間」）。角度空間に寄せるのに要る影は
/// MotionPlayer::start() が先に作ってあり、ここでは IK も FK も呼ばない
/// （200Hz の再生の中で運動学を解かないため）。
///
/// 姿勢は RPY を成分ごとに線形補間する。回転行列の球面補間ではないので、大きく
/// 回す区間では中間の足の向きが厳密な最短経路からずれるが、足裏の傾きは
/// ankleClampJoints() の窓（±35 deg 弱）に収まる範囲でしか使えないので、
/// この範囲では差が出ない。角度を跨ぐ心配が要らないぶん扱いやすい方を採る。
BodyPose blendPose(const BodyPose & a, const BodyPose & b, double u, bool linear);

}  // namespace roboone_motion

#endif  // ROBOONE_MOTION__MOTION_LIBRARY_HPP_
