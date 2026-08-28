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
set(NAVIGATION2D_ACADOS_GENERATED_DIR "" CACHE PATH "Generated Navigation2D acados solver directory")
set(NAVIGATION2D_ACADOS_ROOT "" CACHE PATH "acados installation prefix")
add_library(navigation2d_core STATIC
  application/navigation_config.cc costmap/grid_2d.cc costmap/layered_costmap.cc
  planning/search_core.cc planning/collision_checker.cc planning/obstacle_heuristic.cc planning/grid_search.cc
  planning/motion_primitives.cc planning/path_smoother.cc planning/dijkstra_planner.cc planning/astar_planner.cc
  planning/state_lattice_planner.cc
  planning/theta_star_planner.cc planning/planner_factory.cc
  control/regulated_pure_pursuit.cc control/dwa_controller.cc control/mppi_controller.cc control/dynamic_safety.cc
  control/mpc_controller.cc
  application/navigation_system.cc)
target_include_directories(navigation2d_core PUBLIC "${PROJECT_SOURCE_DIR}/..")
target_link_libraries(navigation2d_core PUBLIC Eigen3::Eigen Sophus::Sophus yaml-cpp)
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
add_executable(layered_costmap_test test/layered_costmap_test.cc)
target_link_libraries(layered_costmap_test PRIVATE navigation2d_core)
add_test(NAME layered_costmap_test COMMAND layered_costmap_test)
add_executable(mppi_controller_test test/mppi_controller_test.cc)
target_link_libraries(mppi_controller_test PRIVATE navigation2d_core)
add_test(NAME mppi_controller_test COMMAND mppi_controller_test)
add_executable(mpc_controller_test test/mpc_controller_test.cc)
target_link_libraries(mpc_controller_test PRIVATE navigation2d_core)
if(NAVIGATION2D_ACADOS_GENERATED_DIR)
  target_compile_definitions(mpc_controller_test PRIVATE NAVIGATION2D_TEST_ACADOS=1)
endif()
add_test(NAME mpc_controller_test COMMAND mpc_controller_test)
add_executable(dynamic_safety_test test/dynamic_safety_test.cc)
target_link_libraries(dynamic_safety_test PRIVATE navigation2d_core)
add_test(NAME dynamic_safety_test COMMAND dynamic_safety_test)
add_executable(global_planner_test test/global_planner_test.cc)
target_link_libraries(global_planner_test PRIVATE navigation2d_core)
add_test(NAME global_planner_test COMMAND global_planner_test)
