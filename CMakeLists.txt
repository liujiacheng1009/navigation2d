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

add_library(navigation2d_core STATIC
  application/algorithm_registry.cc application/navigation_config.cc costmap/grid_2d.cc costmap/layered_costmap.cc
  planning/search_core.cc planning/incremental_search.cc planning/collision_checker.cc
  planning/obstacle_heuristic.cc planning/grid_search.cc
  planning/motion_primitives.cc planning/path_smoother.cc planning/dijkstra_planner.cc planning/astar_planner.cc
  planning/state_lattice_planner.cc
  planning/theta_star_planner.cc planning/planner_factory.cc
  control/regulated_pure_pursuit.cc control/dwa_controller.cc control/nav2_mppi_core.cc
  control/mppi_controller.cc control/dynamic_safety.cc
  control/safe_corridor.cc control/collision_monitor.cc control/mpc_controller.cc
  configuration/vehicle_config.cc
  exploration/exploration_policy.cc
  exploration/frontier_explorer.cc exploration/region_scheduler.cc
  application/command_safety_pipeline.cc
  application/docking_servo.cc
  application/command_stall_watch.cc application/route_preparation.cc planning/path_geometry.cc
  planning/route_identity.cc planning/path_metrics.cc
  application/navigation_system.cc)
target_include_directories(navigation2d_core PUBLIC "${PROJECT_SOURCE_DIR}/..")
target_include_directories(navigation2d_core SYSTEM PRIVATE "${decomp_util_SOURCE_DIR}/include")
target_link_libraries(navigation2d_core PUBLIC navigation2d_geometry PRIVATE yaml-cpp)
if(NAVIGATION2D_ACADOS_GENERATED_DIR)
  if(NOT NAVIGATION2D_ACADOS_ROOT)
    message(FATAL_ERROR "NAVIGATION2D_ACADOS_ROOT is required with NAVIGATION2D_ACADOS_GENERATED_DIR")
  endif()
  set(NAVIGATION2D_ACADOS_SOLVER
      "${NAVIGATION2D_ACADOS_GENERATED_DIR}/libacados_ocp_solver_navigation2d_mpcc.so")
  if(NOT EXISTS "${NAVIGATION2D_ACADOS_SOLVER}")
    message(FATAL_ERROR "generated acados solver not found: ${NAVIGATION2D_ACADOS_SOLVER}")
  endif()
  target_sources(navigation2d_core PRIVATE control/acados_mpc_backend.cc)
  target_include_directories(navigation2d_core PRIVATE
      "${NAVIGATION2D_ACADOS_GENERATED_DIR}" "${NAVIGATION2D_ACADOS_ROOT}/include"
      "${NAVIGATION2D_ACADOS_ROOT}/include/blasfeo/include"
      "${NAVIGATION2D_ACADOS_ROOT}/include/hpipm/include")
  target_link_libraries(navigation2d_core PUBLIC "${NAVIGATION2D_ACADOS_SOLVER}"
      "${NAVIGATION2D_ACADOS_ROOT}/lib/libacados.so"
      "${NAVIGATION2D_ACADOS_ROOT}/lib/libhpipm.so"
      "${NAVIGATION2D_ACADOS_ROOT}/lib/libblasfeo.so")
else()
  target_sources(navigation2d_core PRIVATE control/acados_mpc_backend_stub.cc)
endif()
target_compile_options(navigation2d_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
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
  application/exploration/entry_target.cc
  application/exploration/entry_coordinator.cc
  application/exploration/recovery_policy.cc
  application/exploration/return_geometry.cc
  application/exploration/return_coordinator.cc)
target_compile_features(navigation2d_exploration_application PUBLIC cxx_std_20)
target_link_libraries(navigation2d_exploration_application PUBLIC navigation2d_core)
target_compile_options(navigation2d_exploration_application PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_library(navigation2d_exploration_io STATIC ros/exploration/artifact_writer.cc)
target_compile_features(navigation2d_exploration_io PUBLIC cxx_std_20)
target_link_libraries(navigation2d_exploration_io PUBLIC navigation2d_core)
target_compile_options(navigation2d_exploration_io PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_subdirectory(test)
