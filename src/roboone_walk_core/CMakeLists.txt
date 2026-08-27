cmake_minimum_required(VERSION 3.14)
project(roboone_walk_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# 歩行計画ライブラリ (ヘッダオンリ / ROS 非依存)
# ---------------------------------------------------------------------------
add_library(roboone_walk_core INTERFACE)
target_include_directories(roboone_walk_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(roboone_walk_core INTERFACE cxx_std_17)

# ---------------------------------------------------------------------------
# 自己検算と軌道ダンプ (Python 版との照合は tools/compare_walk_engines.py)
# ---------------------------------------------------------------------------
add_executable(walk_selftest src/walk_selftest.cpp)
target_link_libraries(walk_selftest roboone_walk_core)
target_compile_options(walk_selftest PRIVATE -O2 -Wall -Wextra)

add_executable(walk_dump src/walk_dump.cpp)
target_link_libraries(walk_dump roboone_walk_core)
target_compile_options(walk_dump PRIVATE -O2 -Wall -Wextra)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS roboone_walk_core EXPORT export_roboone_walk_core)
install(TARGETS walk_selftest walk_dump DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_roboone_walk_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  add_test(NAME walk_selftest COMMAND walk_selftest)
endif()

ament_package()
