cmake_minimum_required(VERSION 3.14)
project(roboone_motion_node)

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
#   motion_library キーフレームモーションの読み込みと再生
# motion ノードとティーチツールが同じものを使う。姿勢の解釈を 2 実装にしない。
# ---------------------------------------------------------------------------
add_library(roboone_motion_core SHARED
  src/servo_map.cpp
  src/motion_library.cpp)
target_include_directories(roboone_motion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(roboone_motion_core PUBLIC cxx_std_17)
target_link_libraries(roboone_motion_core PUBLIC yaml-cpp::yaml-cpp)
ament_target_dependencies(roboone_motion_core PUBLIC roboone_kinematics)
target_compile_options(roboone_motion_core PRIVATE -Wall -Wextra)

# ---------------------------------------------------------------------------
# motion ノード (200Hz ループの本体)
# ---------------------------------------------------------------------------
add_executable(motion_node src/motion_node.cpp)
target_link_libraries(motion_node roboone_motion_core feetech_servo::feetech_servo)
ament_target_dependencies(motion_node
  rclcpp std_msgs geometry_msgs sensor_msgs diagnostic_msgs ament_index_cpp
  roboone_kinematics roboone_walk_core)
target_compile_options(motion_node PRIVATE -O2 -Wall -Wextra)

# ---------------------------------------------------------------------------
# ティーチツール (脱力させた機体の姿勢を config のキーフレームとして捕まえる)
#   ROS ノードではない素の CLI。サーボへの書き込みは起動時のトルク OFF 1 回だけ。
# ---------------------------------------------------------------------------
add_executable(motion_teach src/motion_teach.cpp)
target_link_libraries(motion_teach roboone_motion_core feetech_servo::feetech_servo)
ament_target_dependencies(motion_teach ament_index_cpp roboone_kinematics)
target_compile_options(motion_teach PRIVATE -O2 -Wall -Wextra)

# ---------------------------------------------------------------------------
# install / export
# ---------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)
install(TARGETS roboone_motion_core
  EXPORT export_roboone_motion_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(TARGETS motion_node motion_teach DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_roboone_motion_node HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(roboone_kinematics roboone_walk_core feetech_servo yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
