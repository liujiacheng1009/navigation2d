cmake_minimum_required(VERSION 3.16)
project(navigation2d LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
include(FetchContent)
find_package(Eigen3 REQUIRED)
find_package(Sophus 1.22.10 CONFIG QUIET)
if(NOT Sophus_FOUND)
  set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(sophus
    GIT_REPOSITORY https://github.com/strasdat/Sophus.git
    GIT_TAG 1.22.10
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(sophus)
endif()
find_package(yaml-cpp REQUIRED)
FetchContent_Declare(decomp_util
  GIT_REPOSITORY https://github.com/oscardegroot/DecompUtil.git
  GIT_TAG b0836c7228d19f0fa97282c584b55adf642279da
  SOURCE_SUBDIR navigation2d-no-cmake)
FetchContent_MakeAvailable(decomp_util)
set(NAVIGATION2D_ACADOS_GENERATED_DIR "" CACHE PATH "Generated Navigation2D acados solver directory")
set(NAVIGATION2D_ACADOS_ROOT "" CACHE PATH "acados installation prefix")
add_library(navigation2d_values INTERFACE)
target_compile_features(navigation2d_values INTERFACE cxx_std_20)
target_include_directories(navigation2d_values INTERFACE "${PROJECT_SOURCE_DIR}/..")
target_link_libraries(navigation2d_values INTERFACE Eigen3::Eigen Sophus::Sophus)
add_library(navigation2d_geometry STATIC geometry/robot_geometry.cc)
target_link_libraries(navigation2d_geometry PUBLIC navigation2d_values)
target_compile_options(navigation2d_geometry PRIVATE -Wall -Wextra -Wpedantic -Werror)

include(cmake/libraries.cmake)

add_library(navigation2d_simulation STATIC simulation/navigation_simulator.cc)
target_link_libraries(navigation2d_simulation PUBLIC navigation2d_core)
add_executable(navigation2d_benchmark application/benchmark_runner.cc)
target_link_libraries(navigation2d_benchmark PRIVATE navigation2d_simulation)
enable_testing()

add_library(navigation2d_exploration_application STATIC
  application/exploration/debug_overlay.cc
  application/exploration/navigation_session.cc
  application/exploration/navigation_profile.cc
  application/exploration/planning_snapshot.cc
  application/exploration/route_validator.cc
  application/exploration/navigation_telemetry.cc
  application/exploration/room_visit_ledger.cc
  application/exploration/status_serializer.cc
  application/exploration/mission_lifecycle.cc
  application/exploration/dispatch_policy.cc
  application/exploration/pose_motion_watch.cc
  application/exploration/entry_completion.cc
  application/exploration/tour_execution.cc
  application/exploration/mouth_tour_order.cc
  application/exploration/wall_tip_tour_order.cc
  application/exploration/entry_probe_executor.cc
  application/exploration/entry_crawl_executor.cc
  application/exploration/execution_command.cc
  application/exploration/viewpoint_arrival.cc
  application/exploration/trail_navigation.cc
  application/exploration/retreat_execution.cc
  application/exploration/entry_selection.cc
  application/exploration/selection_exhaustion.cc
  application/exploration/viewpoint_rejections.cc
  application/exploration/viewpoint_rejection_executor.cc
  application/exploration/completion_execution.cc
  application/exploration/entry_target.cc
  application/exploration/entry_coordinator.cc
  application/exploration/recovery_policy.cc
  application/exploration/return_geometry.cc
  application/exploration/return_coordinator.cc)
target_compile_features(navigation2d_exploration_application PUBLIC cxx_std_20)
target_link_libraries(navigation2d_exploration_application PUBLIC navigation2d_core)
target_compile_options(navigation2d_exploration_application PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_library(navigation2d_exploration_io STATIC
  ros/exploration/artifact_writer.cc
  ros/exploration/planning_map_adapter.cc)
target_compile_features(navigation2d_exploration_io PUBLIC cxx_std_20)
target_link_libraries(navigation2d_exploration_io PUBLIC navigation2d_exploration_application)
target_compile_options(navigation2d_exploration_io PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_subdirectory(test)
