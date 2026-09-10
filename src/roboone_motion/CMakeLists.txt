cmake_minimum_required(VERSION 3.14)
project(roboone_motion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  # 200Hz ループなので既定を Release にする。Debug でも回るが、
  # 足首の順変換 (ニュートン法) と IK が効いてくる。
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(feetech_servo REQUIRED)
find_package(roboone_kinematics REQUIRED)
find_package(roboone_walk_core REQUIRED)

# ---------------------------------------------------------------------------
# 共通部分 (ROS 非依存)
#   servo_map      生カウント <-> サーボ角 <-> 関節角 の対応
#   servo_bank     サーボ層。2 バスの開閉・スレッド・トルク・生カウントの授受
#   pose_codec     変換層。BodyPose <-> 生カウント (Σ_U / Σ_B の境界)
#   motion_config  設定層。YAML の読み込みと起動時の門
#   motion_library キーフレームモーションの読み込みと再生
#   motion_control 生成層。状態機械・歩行・技の再生
#
# motion ノードとティーチツールが同じものを使う。姿勢の解釈を 2 実装にしない。
# **ROS を知らないので motion_selftest から実機なしで叩ける。**
# ---------------------------------------------------------------------------
add_library(roboone_motion_core SHARED
  src/servo_map.cpp
  src/servo_bank.cpp
  src/pose_codec.cpp
  src/motion_config.cpp
  src/motion_library.cpp
  src/motion_control.cpp)
target_include_directories(roboone_motion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(roboone_motion_core PUBLIC cxx_std_17)
target_link_libraries(roboone_motion_core
  PUBLIC yaml-cpp::yaml-cpp feetech_servo::feetech_servo)
ament_target_dependencies(roboone_motion_core PUBLIC roboone_kinematics roboone_walk_core)
target_compile_options(roboone_motion_core PRIVATE -Wall -Wextra)

# ---------------------------------------------------------------------------
# motion ノード (200Hz ループの本体)
# ---------------------------------------------------------------------------
add_executable(motion_node src/motion_node.cpp)
target_link_libraries(motion_node roboone_motion_core)
ament_target_dependencies(motion_node
  rclcpp std_msgs geometry_msgs sensor_msgs diagnostic_msgs ament_index_cpp)
target_compile_options(motion_node PRIVATE -O2 -Wall -Wextra)

# ---------------------------------------------------------------------------
# ティーチツール (脱力させた機体の姿勢を config のキーフレームとして捕まえる)
#   ROS ノードではない素の CLI。サーボへの書き込みは起動時のトルク OFF 1 回だけ。
# ---------------------------------------------------------------------------
add_executable(motion_teach src/motion_teach.cpp)
target_link_libraries(motion_teach roboone_motion_core)
ament_target_dependencies(motion_teach ament_index_cpp)
target_compile_options(motion_teach PRIVATE -O2 -Wall -Wextra)

# ---------------------------------------------------------------------------
# 自己検算 (実機もサーボも要らない)
#   角の 3 つの表し方・胴体の前傾の等価性・脚の角度書きの読み込みと補間
# ---------------------------------------------------------------------------
add_executable(motion_selftest src/motion_selftest.cpp)
target_link_libraries(motion_selftest roboone_motion_core)
ament_target_dependencies(motion_selftest ament_index_cpp)
target_compile_options(motion_selftest PRIVATE -O2 -Wall -Wextra)

# ---------------------------------------------------------------------------
# install / export
# ---------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)
install(TARGETS roboone_motion_core
  EXPORT export_roboone_motion
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(TARGETS motion_node motion_teach motion_selftest DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_roboone_motion HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(roboone_kinematics roboone_walk_core feetech_servo yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # 原点ファイルは feetech_servo の share から読むので、あちらが install されて
  # いないと動かない (colcon test は依存を先に install するので通常は満たされる)。
  #
  # ★--strict は付けない。ここで見たいのはコードの検算 ([1]-[5]) で、設定の門 ([6])
  #   が赤いかどうかは config の調整状況の話。混ぜると「テストはいつも赤いもの」に
  #   なって、コードの赤に気付けなくなる。config を詰めるときは手で --strict を付ける。
  add_test(NAME motion_selftest COMMAND motion_selftest)
endif()

ament_package()
