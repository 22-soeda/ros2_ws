cmake_minimum_required(VERSION 3.14)
project(roboone_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# 運動学ライブラリ (ヘッダオンリ / ROS 非依存)
# ---------------------------------------------------------------------------
add_library(roboone_kinematics INTERFACE)
target_include_directories(roboone_kinematics INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(roboone_kinematics INTERFACE cxx_std_17)

# ---------------------------------------------------------------------------
# 自己検算 (FK/IK の往復・導出との照合・速度)
# ---------------------------------------------------------------------------
add_executable(leg_selftest src/leg_selftest.cpp)
target_link_libraries(leg_selftest roboone_kinematics)
target_compile_options(leg_selftest PRIVATE -O2)

add_executable(leg_dump src/leg_dump.cpp)
target_link_libraries(leg_dump roboone_kinematics)
target_compile_options(leg_dump PRIVATE -O2)

# 可視化 (roboone_motion/viz/serve_leg3d.py) が起動する FK/IK サービス
add_executable(leg_service src/leg_service.cpp)
target_link_libraries(leg_service roboone_kinematics)
target_compile_options(leg_service PRIVATE -O2)

# ---------------------------------------------------------------------------
# 膝 4 節リンク（docs/膝4節リンク導出.pdf）
#   knee_selftest : 受け入れ基準 1〜8 + σ の一意性 + 脚 IK との繋ぎこみ
#   knee_dump     : Python 参照実装との突き合わせ用（scripts/crosscheck_knee.py）
# ---------------------------------------------------------------------------
add_executable(knee_selftest src/knee_selftest.cpp)
target_link_libraries(knee_selftest roboone_kinematics)
target_compile_options(knee_selftest PRIVATE -O2)

add_executable(knee_dump src/knee_dump.cpp)
target_link_libraries(knee_dump roboone_kinematics)
target_compile_options(knee_dump PRIVATE -O2)

# ---------------------------------------------------------------------------
# 足首パラレルリンク（docs/足首パラレルリンク導出.pdf）
#   ankle_selftest : §8 の検算 + 文書が本文で挙げている数値との照合
#   ankle_dump     : 中間量を出す確認用（CAD の値を入れたあとの点検に使う）
# ---------------------------------------------------------------------------
add_executable(ankle_selftest src/ankle_selftest.cpp)
target_link_libraries(ankle_selftest roboone_kinematics)
target_compile_options(ankle_selftest PRIVATE -O2)

add_executable(ankle_dump src/ankle_dump.cpp)
target_link_libraries(ankle_dump roboone_kinematics)
target_compile_options(ankle_dump PRIVATE -O2)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS leg_selftest leg_dump leg_service knee_selftest knee_dump
  ankle_selftest ankle_dump
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS roboone_kinematics EXPORT export_roboone_kinematics)
ament_export_targets(export_roboone_kinematics HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 自己検算をそのまま ctest に載せる (姿勢数を減らして高速に)
  add_test(NAME leg_selftest COMMAND leg_selftest -n 3000)
  add_test(NAME knee_selftest COMMAND knee_selftest)
  add_test(NAME ankle_selftest COMMAND ankle_selftest -n 2000)
endif()

ament_package()
