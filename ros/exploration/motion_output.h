#pragma once
#include <memory>
#include <string>

#include "navigation2d/application/exploration/motion_output.h"
#include "rclcpp/node.hpp"
namespace navigation2d::exploration_ros {
// Same topic, queue depth and planar-to-ROS message mapping as the original
// node. A topic argument permits isolated adapter tests without /cmd_vel.
[[nodiscard]] std::unique_ptr<exploration::MotionOutput> MakeRosMotionOutput(
    rclcpp::Node&, const std::string& topic = "/cmd_vel");
}  // namespace navigation2d::exploration_ros
