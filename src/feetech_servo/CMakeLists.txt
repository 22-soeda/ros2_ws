cmake_minimum_required(VERSION 3.14)
project(feetech_servo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---------------------------------------------------------------------------
# 同梱した Feetech 公式 SCServo SDK (静的ライブラリ)
# ---------------------------------------------------------------------------
file(GLOB SCSERVO_SOURCES CONFIGURE_DEPENDS vendor/scservo/src/*.cpp)
add_library(scservo STATIC ${SCSERVO_SOURCES})
target_include_directories(scservo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/scservo/include>
  $<INSTALL_INTERFACE:include>)
# SDK の .cpp は兄弟ヘッダを bare 名 ("SCS.h" 等) で include するため、
# scservo/ を private include に加える。
target_include_directories(scservo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/scservo/include/scservo)
target_compile_features(scservo PUBLIC cxx_std_17)
set_target_properties(scservo PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# feetech_servo ヘルパライブラリ (SDK をラップ。ROS 非依存の純粋ライブラリ)
# ---------------------------------------------------------------------------
add_library(feetech_servo SHARED
  src/feetech_bus.cpp
  src/feetech_manager.cpp)
target_include_directories(feetech_servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(feetech_servo PUBLIC cxx_std_17)
# SDK は内部実装にだけ必要 (公開ヘッダは前方宣言で隠蔽) なので PRIVATE。
target_link_libraries(feetech_servo PRIVATE scservo)

# ---------------------------------------------------------------------------
# デモ node (2バス並列で閉ループを回す)
# ---------------------------------------------------------------------------
add_executable(feetech_demo_node src/feetech_demo_node.cpp)
target_link_libraries(feetech_demo_node feetech_servo)
ament_target_dependencies(feetech_demo_node rclcpp sensor_msgs)

# ---------------------------------------------------------------------------
# 実機テストツール (ROS非依存の素のCLI。実機がないと意味がないので ament テストには載せない)
#   feetech_scan_test : 1Mbps のバス上の全サーボを列挙して情報を出す
#   feetech_goto_test : 定義済み全軸を一斉に約30秒かけて 2047 へ動かす
#   feetech_shell     : 1軸ずつ手で操作する対話CLI（bus/ID を選んで ping/pos/on/off/go）
#   feetech_set_limits: 設定ファイル(YAML)の角度リミットを全軸へ一括書き込み
#   feetech_calibrate_home: 1軸ずつ手で原点を合わせ、生カウントを YAML に保存する
# ---------------------------------------------------------------------------
add_executable(feetech_scan_test test/feetech_scan_test.cpp)
# scan は SMS_STS の EEPROM アドレス定義 (SMS_STS_*) を直接使うので SDK にもリンクする。
target_link_libraries(feetech_scan_test feetech_servo scservo)

add_executable(feetech_goto_test test/feetech_goto_test.cpp)
target_link_libraries(feetech_goto_test feetech_servo Threads::Threads)

# shell も EEPROM アドレス定義 (SMS_STS_*) を使うので SDK にリンクする。
add_executable(feetech_shell test/feetech_shell.cpp)
target_link_libraries(feetech_shell feetech_servo scservo)

# set_limits は EEPROM アドレス定義と YAML パーサ、既定設定の場所解決に ament_index を使う。
add_executable(feetech_set_limits test/feetech_set_limits.cpp)
target_link_libraries(feetech_set_limits feetech_servo scservo yaml-cpp::yaml-cpp)
ament_target_dependencies(feetech_set_limits ament_index_cpp)

# calibrate_home も軸一覧(YAML)と TORQUE_ENABLE のアドレス定義を使う。
add_executable(feetech_calibrate_home test/feetech_calibrate_home.cpp)
target_link_libraries(feetech_calibrate_home feetech_servo scservo yaml-cpp::yaml-cpp)
ament_target_dependencies(feetech_calibrate_home ament_index_cpp)

# ---------------------------------------------------------------------------
# install / export
# ---------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)

install(TARGETS feetech_servo scservo
  EXPORT export_feetech_servo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(TARGETS feetech_demo_node feetech_scan_test feetech_goto_test feetech_shell
  feetech_set_limits feetech_calibrate_home
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_feetech_servo HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
