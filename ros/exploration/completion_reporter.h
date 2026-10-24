#pragma once
#include "navigation2d/application/exploration/completion_execution.h"
#include "rclcpp/logger.hpp"
namespace navigation2d::exploration_ros {
void ReportCompletion(const rclcpp::Logger&, const exploration::CompletionNotice&);
}  // namespace navigation2d::exploration_ros
