#include "navigation2d/ros/exploration/navigation_reporter.h"

#include "navigation2d/application/exploration/goal_execution_watch.h"
#include "rclcpp/logging.hpp"

namespace navigation2d::exploration_ros {
void ReportRouteProgressRecovery(const rclcpp::Logger& logger, const NavigationState& state,
                                 Twist2d measured_velocity) {
  RCLCPP_WARN(
      logger,
      "Route progress recovery: phase=%d arc=%.2f/%.2f requested=(%.3f,%.3f) published=(%.3f,%.3f) "
      "controller_motion=%s safety_stop=%s rpp_stage=%d controller_maneuver=%d intentional_stop=%s "
      "monitor_action=%d ttc=%.3f velocity=(%.3f,%.3f) path_signature=%llu planner_error=%s",
      static_cast<int>(state.phase), state.path_progress_m, state.global_path_length_m,
      state.requested_command.linear, state.requested_command.angular,
      state.published_command.linear, state.published_command.angular,
      state.controller_commanded_motion ? "yes" : "no", state.safety_stopped_motion ? "yes" : "no",
      state.controller_diagnostics.fallback_level,
      static_cast<int>(state.controller_diagnostics.maneuver),
      state.controller_diagnostics.intentional_stop ? "yes" : "no",
      static_cast<int>(state.collision_monitor_action), state.minimum_ttc_s,
      measured_velocity.linear, measured_velocity.angular,
      static_cast<unsigned long long>(state.path_signature),
      state.planning_failure_reason.empty() ? "none" : state.planning_failure_reason.c_str());
}
void ReportNavigationFailure(const rclcpp::Logger& logger, const NavigationState& state,
                             const exploration::GoalExecutionWatch& execution,
                             const exploration::GoalExecutionClock& clock) {
  RCLCPP_WARN(
      logger,
      "Navigation2D frontier failed: status=%s phase=%d elapsed=%.1f replans=%d path=%.2f "
      "requested=(%.3f,%.3f) published=(%.3f,%.3f) controller_maneuver=%d intentional_stop=%s "
      "monitor_action=%d ttc=%.3f path_signature=%llu planner_error=%s",
      state.status == navigation2d::NavigationStatus::kBlocked ? "blocked" : "timeout",
      static_cast<int>(state.phase), execution.Elapsed(clock), state.replans,
      state.global_path_length_m, state.requested_command.linear, state.requested_command.angular,
      state.published_command.linear, state.published_command.angular,
      static_cast<int>(state.controller_diagnostics.maneuver),
      state.controller_diagnostics.intentional_stop ? "yes" : "no",
      static_cast<int>(state.collision_monitor_action), state.minimum_ttc_s,
      static_cast<unsigned long long>(state.path_signature),
      state.planning_failure_reason.empty() ? "none" : state.planning_failure_reason.c_str());
}
}  // namespace navigation2d::exploration_ros
