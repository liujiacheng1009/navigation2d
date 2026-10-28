#include "navigation2d/ros/exploration/navigation_reporter.h"

#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <string>

#include "navigation2d/application/exploration/goal_execution_watch.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
namespace {
std::string captured;
int writes = 0;
void Capture(const rcutils_log_location_t*, int severity, const char* name,
             rcutils_time_point_value_t, const char* format, va_list* arguments) {
  assert(severity == RCUTILS_LOG_SEVERITY_WARN);
  assert(std::string(name) == "navigation_reporter_test");
  char buffer[2048];
  va_list copy;
  va_copy(copy, *arguments);
  const int size = std::vsnprintf(buffer, sizeof(buffer), format, copy);
  va_end(copy);
  assert(size >= 0 && static_cast<std::size_t>(size) < sizeof(buffer));
  captured.assign(buffer, static_cast<std::size_t>(size));
  ++writes;
}
class Clock final : public navigation2d::exploration::GoalExecutionClock {
 public:
  mutable int reads = 0;
  std::int64_t ExecutionNowNs() const override {
    ++reads;
    return 5000000000LL;
  }
};
}  // namespace
int main(int argc, char** argv) {
  using namespace navigation2d;
  rclcpp::init(argc, argv);
  const auto logger = rclcpp::get_logger("navigation_reporter_test");
  const auto old_handler = rcutils_logging_get_output_handler();
  const int old_level = rcutils_logging_get_logger_level(logger.get_name());
  rcutils_logging_set_output_handler(Capture);
  assert(rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_FATAL) ==
         RCUTILS_RET_OK);
  Clock clock;
  exploration::GoalExecutionWatch execution;
  execution.Start(2000000000LL, 2000000000LL);
  NavigationState state;
  state.status = NavigationStatus::kBlocked;
  state.replans = 2;
  state.global_path_length_m = 4.2;
  state.requested_command = {.1, -.2};
  state.controller_diagnostics.maneuver = ControllerManeuver::kTracking;
  state.collision_monitor_action = CollisionMonitorAction::kStop;
  state.minimum_ttc_s = .125;
  state.path_signature = 42;
  state.planning_failure_reason = "example";
  exploration_ros::ReportNavigationFailure(logger, state, execution, clock);
  assert(clock.reads == 0 && writes == 0);
  assert(rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_WARN) ==
         RCUTILS_RET_OK);
  exploration_ros::ReportNavigationFailure(logger, state, execution, clock);
  assert(clock.reads == 1 && writes == 1);
  assert(captured ==
         "Navigation2D frontier failed: status=blocked phase=0 elapsed=3.0 replans=2 "
         "path=4.20 requested=(0.100,-0.200) published=(0.000,0.000) controller_maneuver=0 "
         "intentional_stop=no monitor_action=2 ttc=0.125 path_signature=42 planner_error=example");
  state.status = NavigationStatus::kNavigating;
  state.planning_failure_reason.clear();
  exploration_ros::ReportNavigationFailure(logger, state, execution, clock);
  assert(clock.reads == 2 && writes == 2);
  assert(captured.find("status=timeout") != std::string::npos);
  assert(captured.ends_with("planner_error=none"));
  rcutils_logging_set_output_handler(old_handler);
  assert(rcutils_logging_set_logger_level(logger.get_name(), old_level) == RCUTILS_RET_OK);
  rclcpp::shutdown();
}
