#pragma once
#include "navigation2d/application/navigation_state.h"
#include "rclcpp/logger.hpp"

namespace navigation2d::exploration_ros {
void ReportRouteProgressRecovery(const rclcpp::Logger& logger, const NavigationState& state,
                                 Twist2d measured_velocity);
}  // namespace navigation2d::exploration_ros
