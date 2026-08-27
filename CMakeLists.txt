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
add_library(navigation2d_core STATIC
  application/navigation_config.cc costmap/grid_2d.cc costmap/layered_costmap.cc
  planning/grid_search.cc planning/dijkstra_planner.cc planning/astar_planner.cc
  planning/theta_star_planner.cc planning/planner_factory.cc
  control/regulated_pure_pursuit.cc control/dwa_controller.cc application/navigation_system.cc)
target_include_directories(navigation2d_core PUBLIC "${PROJECT_SOURCE_DIR}/..")
target_link_libraries(navigation2d_core PUBLIC Eigen3::Eigen Sophus::Sophus yaml-cpp)
target_compile_options(navigation2d_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_library(navigation2d_simulation STATIC simulation/navigation_simulator.cc)
target_link_libraries(navigation2d_simulation PUBLIC navigation2d_core)
add_executable(navigation2d_benchmark application/benchmark_runner.cc)
target_link_libraries(navigation2d_benchmark PRIVATE navigation2d_simulation)
enable_testing()
add_executable(layered_costmap_test test/layered_costmap_test.cc)
target_link_libraries(layered_costmap_test PRIVATE navigation2d_core)
add_test(NAME layered_costmap_test COMMAND layered_costmap_test)
