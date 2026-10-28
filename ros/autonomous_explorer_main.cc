#include <cstdio>
#include <exception>

#include "navigation2d/ros/exploration/explorer_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(navigation2d::exploration_ros::MakeAutonomousExplorer());
  } catch (const std::exception& error) {
    std::fprintf(stderr, "autonomous explorer failed: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
