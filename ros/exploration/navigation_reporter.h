#pragma once
#include "navigation2d/application/navigation_state.h"
#include "rclcpp/logger.hpp"

namespace navigation2d::exploration {
class GoalExecutionWatch;
class GoalExecutionClock;
}  // namespace navigation2d::exploration
namespace navigation2d::exploration_ros {
// Clock read stays inside the enabled log branch, matching the node's
// original RCLCPP_WARN argument laziness.
void ReportNavigationFailure(const rclcpp::Logger&, const NavigationState&,
                             const exploration::GoalExecutionWatch&,
                             const exploration::GoalExecutionClock&);
void ReportRouteProgressRecovery(const rclcpp::Logger& logger, const NavigationState& state,
                                 Twist2d measured_velocity);
}  // namespace navigation2d::exploration_ros
