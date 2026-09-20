// 設定層 — YAML の読み込みと、起動時に通す「門」。
//
// ===========================================================================
// 門を先に通す理由
// ===========================================================================
// 解けない config を実機で踏むと、**「動かない」のか「解けていない」のかが現場で
// 切り分けられない。** どれも起動時に分かる種類の食い違いなので、走り出す前に
// 言葉で出す。
//
//   checkPoseReachable    ホーム姿勢が IK で解けるか
//   checkMotionLegServo   角度書きのキーフレームが servo_limits.yaml の窓に入るか
//   checkGait             遊脚が本当に床へ届くか（降下は td_speed_max で飽和する）
//   checkStance           歩行の立位（= ホーム姿勢の足）と計画上の足間隔の関係を言う
//   checkWalkEnvelope     歩行が指令しうる足先の箱が IK の到達域に収まるか
//
// 静歩行 (walk_mode:=static) では、上の歩行の 3 つの代わりに次の 3 つを通す。
//
//   checkStaticGait           static_gait.yaml が静歩行として成り立つか
//   checkStaticStance         計画の足間隔とホーム姿勢の足が揃っているか (静的な余裕)
//   checkStaticWalkEnvelope   計画した軌道の全時刻で、足先に脚が届くか
//
// checkWalkEnvelope は roboone_walk_core/src/gait_from_kinematics.cpp の逆向き。
// あちらは IK の到達域から gait.yaml の値を**決める**、こちらは入っている値で
// 本当に届くかを**確かめる**。片方だけ手で書き換えたときに気付くための門。
//
// ===========================================================================
// ROS を知らない
// ===========================================================================
// 出来事は EventQueue へ積むだけ。おかげで motion_selftest から実機なしで
// 全部の門を通せる（config を書き換えたら、まずそこで確かめられる）。
#ifndef ROBOONE_MOTION__MOTION_CONFIG_HPP_
#define ROBOONE_MOTION__MOTION_CONFIG_HPP_

#include <string>

#include "roboone_motion/body_pose.hpp"
#include "roboone_motion/event.hpp"
#include "roboone_motion/motion_library.hpp"
#include "roboone_motion/servo_map.hpp"
#include "roboone_motion/walk_planner.hpp"
#include "roboone_walk_core/gait_params.hpp"
#include "roboone_walk_core/static_walk_engine.hpp"

namespace roboone_motion
{

namespace rwc = roboone_walk_core;

/// パラメータで渡されたパスが空なら、パッケージ既定へ落とす。
///
/// 空になるのは大抵「別のノード向けの params ファイルが流れ込んだ」とき
/// （launch の設定値は兄弟の include へ漏れるので、引数名が衝突すると起こる。
/// 2026-08-29 に teleop の ps5_dualsense.yaml が motion へ渡って実際に踏んだ）。
/// **空パスで即死するより、既定へ落として理由を言うほうがいい。**
void fallbackPath(std::string & path, const std::string & def, const char * what, EventQueue & ev);

/// gait.yaml を読む。**読めなくても false にしない**（gait_params.hpp の既定値で走れる）。
///
/// 打ち間違いを黙って既定値に落とさないため、知らないキーは警告する
/// （Python 版 GaitParams.from_yaml が KeyError で落ちるのと同じ意図）。
void loadGait(const std::string & path, rwc::GaitParams & out, EventQueue & ev);

/// home_pose.yaml を読む。**足裏の位置姿勢で書かれている**（関節角ではない）。
///
/// 狙いは「胴体が直立・足裏が水平・骨盤が所定の高さ」なので、そこに至る関節角は
/// IK が出せばよい。関節角で書くと AXIS_FLIP（サーボの回転方向）が絡んで左右で
/// 同じ数値が同じ姿勢にならないが、足裏の位置姿勢なら鏡像は y の符号だけで済む。
///
/// body_pitch は foot: の外に置いてある。足裏そのものの性質ではなく**骨盤の姿勢**
/// だから（歩行・技・ホームの全部に一様に掛かる）。
///
/// **立位は歩行の計画器 (walk.mode) ごとに持てる。** yaml の `walk_mode: <dynamic|static>:
/// foot:` に書いたキー (height / x / y / rpy) だけが `foot:` を上書きする。モードは
/// 起動時に 1 回決まるので、HOLD・home・歩行は全部そのモードの立位に揃う。
/// 骨盤高さの突き合わせも、選んだモードの計画の z_c (walk.planZc()) とだけ行う。
bool loadHomePose(
  const std::string & path, const ServoMap & map, const WalkSetup & walk,
  BodyPose & out, double & body_pitch, EventQueue & ev, std::string & err);
/// 動歩行で読む（従来の呼び方）。
bool loadHomePose(
  const std::string & path, const ServoMap & map, const rwc::GaitParams & gait,
  BodyPose & out, double & body_pitch, EventQueue & ev, std::string & err);

// --- 門 -------------------------------------------------------------------

/// 姿勢が IK で解けるかを見る。
void checkPoseReachable(
  const ServoMap & map, const BodyPose & pose, const char * what, EventQueue & ev);

/// 角度書き (R_leg / L_leg) の枚が servo_limits.yaml の窓に収まっているかを見る。
///
/// 角度書きは IK も FK も通らずにサーボへ出るので、**この窓が唯一の歯止め**になる。
/// 窓の外を書くと再生時に黙って丸められ、実機では「その軸だけ途中で止まる」形で
/// 出てくる。足裏書きに対する checkPoseReachable と同じ役目をこちらで果たす。
void checkMotionLegServo(const ServoMap & map, const MotionLibrary & lib, EventQueue & ev);

/// 遊脚が本当に床へ届くかを実軌道で見る。
///
/// **降下は td_speed_max で飽和する**ので、swing_ratio に書いた値がそのまま両足支持に
/// なるわけではない。黙って「浮いたまま支持脚が交代する」が起きるのがいちばん困る。
void checkGait(const rwc::GaitParams & gait, EventQueue & ev);

/// 歩行の立位と計画上の足間隔の関係を言う。
///
/// 歩行の足はホーム姿勢の足に揃えてある（MotionController::configure()）。実機の足の
/// 位置を決めるのは home_pose.yaml の foot で、gait.yaml の foot_spacing は横の重心経路
/// にだけ効く。計画が実機の足より狭いと、骨盤が支持足の上まで来ないので警告する。
void checkStance(const rwc::GaitParams & gait, const BodyPose & home, EventQueue & ev);

/// 歩行が実際に指令しうる足先の範囲が、脚 IK の到達域に収まっているかを見る。
///
/// 見る箱は、ホーム姿勢の足 (= 歩行の立位) からの
///   x  ±step_clamp_x                       前後の着地点クランプ
///   y  +step_clamp_out / -step_clamp_in     外側 / 内側 (右脚基準)
///   z  0 .. +swing_height                   遊脚の高さ
/// の 8 隅 + 中心。足裏はホーム姿勢の向きのまま (walk_core は平行移動のみ)。
/// ★骨盤の横振りは含まない。足踏みでも遊脚は骨盤から 130mm 以上開くので、ここが
///   通っても歩行中に届くとは限らない（実際に出した足先は tickWalk が見張る）。
void checkWalkEnvelope(
  const ServoMap & map, const rwc::GaitParams & gait, const BodyPose & home,
  double body_pitch, EventQueue & ev);

// --- 静歩行 ---------------------------------------------------------------

/// static_gait.yaml を読む。loadGait と同じく、読めなくても既定値で走り、知らないキーは警告する。
void loadStaticGait(const std::string & path, rwc::StaticGaitParams & out, EventQueue & ev);

/// 静歩行の設定が成り立つか (遊脚が床に着くか・重心の横ずらしが足裏に収まるか)。
/// 1 歩の時間と全速前進の速さも言う。
void checkStaticGait(const rwc::StaticGaitParams & stat, EventQueue & ev);

/// 計画の立位とホーム姿勢の足の関係から、振り出し中の重心が支持足の中心から
/// どれだけずれるかを出す。
///
/// 歩行の足はホーム姿勢の足へ平行移動する (walkStanceOffset)。静歩行の計画は
/// 「重心 = 支持足の中心 + com_offset_y」なので、W/2 と foot.y の差や foot.x は
/// そのまま重心のずれになる。静的な余裕 (足裏の縁まで) が zmp_tol 以下ならエラー。
void checkStaticStance(
  const rwc::StaticGaitParams & stat, const BodyPose & home, EventQueue & ev);

/// 静歩行の計画を前後・左右・斜め・切り返しの指令で回し、**全時刻の足先**を IK と
/// 機構層に通す。機構として届かない時刻があればエラー。
///
/// 箱の隅を見る checkWalkEnvelope と違い、骨盤の横振り (重心を支持足の上へ運ぶ)
/// を含めて見る。静歩行は遊脚が骨盤から横へ 140mm 以上開くので、箱では足りない。
/// 指令の組は roboone_viz/static_reach.py と同じ (足上げの表はあちらで作る)。
void checkStaticWalkEnvelope(
  const ServoMap & map, const WalkSetup & walk, const BodyPose & home,
  double body_pitch, EventQueue & ev);

// --- 安定化 ---------------------------------------------------------------

/// 板の補正（両足裏を平面ごと回す。stab.board = true）を入れても脚が届くかを見る。
///
/// 計画を 11 通りの指令で回し、各時刻で板を上限の 8 通り（ロール / ピッチ 1 軸ずつと
/// 隅の 4 つ）に回して IK と機構層に通す。板は脚を (W/2)·sin u ずつ伸び縮みさせるので、
/// 足首パラレルリンクが先に尽きる。届かない時刻があれば、どこまでなら通るかを言う
/// （その周期は PoseCodec が板を縮めて出すので、指令そのものは止まらない）。
/// **動歩行・静歩行のどちらでも呼べる**（WalkSetup の mode で計画器が決まる）。
void checkBoardEnvelope(
  const ServoMap & map, const WalkSetup & walk, const BodyPose & home,
  double body_pitch, double clamp, EventQueue & ev);

}  // namespace roboone_motion

#endif  // ROBOONE_MOTION__MOTION_CONFIG_HPP_
