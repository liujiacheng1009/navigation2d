#pragma once
#include <string>

#include "navigation2d/application/navigation_system.h"
namespace navigation2d::exploration {
struct NavigationTelemetryState {
  std::string backend = "not-run";
  std::string nav_status = "idle";
  std::string nav_phase = "align_to_path";
  std::string maneuver = "tracking";
  std::string monitor_action = "none";
  double path_progress_m = 0.;
  double path_length_m = 0.;
  double path_max_curvature = 0.;
  double cmd_linear = 0.;
  double cmd_angular = 0.;
  double requested_linear = 0.;
  double requested_angular = 0.;
  double published_linear = 0.;
  double published_angular = 0.;
  double measured_linear = 0.;
  double measured_angular = 0.;
  double controller_solve_us = 0.;
  double minimum_ttc_s = 0.;
  double planner_elapsed_s = 0.;
  bool controller_commanded = false;
  bool safety_stopped = false;
  bool intentional_stop = false;
  int replans = 0;
  int incremental_replans = 0;
  int recoveries = 0;
  int emergency_stops = 0;
  int fallback_level = 0;
  int planner_expansions = 0;
};
class NavigationTelemetry {
 public:
  void Capture(const NavigationState& state, const Twist2d& measured_velocity);
  const NavigationTelemetryState& state() const { return state_; }

 private:
  NavigationTelemetryState state_;
};
}  // namespace navigation2d::exploration
