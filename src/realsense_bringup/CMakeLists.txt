cmake_minimum_required(VERSION 3.14)
project(realsense_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ---------------------------------------------------------------------------
# 実機テストツール
#   カメラが繋がっていないと意味がないので ament のテスト (ctest) には載せず、
#   手で走らせる CLI にしてある（feetech_servo の test/ と同じ方針）。
#   どれも判定が通れば終了コード 0、外れれば 1 を返す。
#
#   rs_stream_test : 全トピックの周期・ジッタ・遅延・中身をまとめて表にする
#   rs_imu_test    : 静止時の重力・ジャイロバイアス・ノイズを測る
#   rs_depth_test  : depth 画像の有効画素率と距離マップを出す
# ---------------------------------------------------------------------------
add_executable(rs_stream_test test/rs_stream_test.cpp)
ament_target_dependencies(rs_stream_test rclcpp sensor_msgs)

add_executable(rs_imu_test test/rs_imu_test.cpp)
ament_target_dependencies(rs_imu_test rclcpp sensor_msgs)

add_executable(rs_depth_test test/rs_depth_test.cpp)
ament_target_dependencies(rs_depth_test rclcpp sensor_msgs)

install(TARGETS rs_stream_test rs_imu_test rs_depth_test
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
