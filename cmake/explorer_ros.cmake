# Optional ROS assembly. The caller first adds navigation2d's application/IO
# targets; importing this file alone does not require a ROS installation.
include_guard(GLOBAL)

function(navigation2d_add_explorer_ros navigation_source)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  add_library(navigation2d_explorer_ros STATIC
    "${navigation_source}/ros/autonomous_explorer_node.cc"
    "${navigation_source}/ros/exploration/explorer_subscriptions.cc"
    "${navigation_source}/ros/exploration/motion_output.cc"
    "${navigation_source}/ros/exploration/explorer_parameters.cc"
    "${navigation_source}/ros/exploration/completion_reporter.cc"
    "${navigation_source}/ros/exploration/entry_reporter.cc"
    "${navigation_source}/ros/exploration/navigation_reporter.cc"
    "${navigation_source}/ros/exploration/pose_feedback.cc"
    "${navigation_source}/ros/exploration/debug_output.cc"
    "${navigation_source}/ros/exploration/debug_reporter.cc"
    "${navigation_source}/ros/exploration/debug_paths.cc"
    "${navigation_source}/ros/exploration/map_observation.cc")
  target_link_libraries(navigation2d_explorer_ros PUBLIC
    navigation2d_exploration_application navigation2d_exploration_io
    rclcpp::rclcpp ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS})
  target_compile_options(navigation2d_explorer_ros PRIVATE -Wall -Wextra -Wpedantic)
  add_executable(sweepnav_autonomous_explorer
    "${navigation_source}/ros/autonomous_explorer_main.cc")
  target_link_libraries(sweepnav_autonomous_explorer PRIVATE navigation2d_explorer_ros)
  target_compile_options(sweepnav_autonomous_explorer PRIVATE -Wall -Wextra -Wpedantic)
endfunction()

# Build-only integration probes: explicit runners own DDS/network isolation.
function(navigation2d_add_explorer_ros_tests navigation_source)
  add_executable(explorer_node_factory_test
    "${navigation_source}/test/explorer_node_factory_test.cc")
  target_link_libraries(explorer_node_factory_test PRIVATE navigation2d_explorer_ros)
  target_compile_options(explorer_node_factory_test PRIVATE
    -Wall -Wextra -Wpedantic -Werror -UNDEBUG)

  # Build-only here: execution requires the network/IPC-isolated test runner.
  add_executable(explorer_ros_loop_test "${navigation_source}/test/explorer_ros_loop_test.cc")
  target_link_libraries(explorer_ros_loop_test PRIVATE navigation2d_explorer_ros yaml-cpp)
  target_compile_options(explorer_ros_loop_test PRIVATE
    -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
endfunction()
