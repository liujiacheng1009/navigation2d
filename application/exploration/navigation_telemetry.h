#pragma once
#include <cstdint>
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
struct ControllerCommandCounts {
  std::uint64_t acados = 0, mppi = 0, rpp = 0;
};
class NavigationTelemetry {
 public:
  // Capture is observational. Count only when execution reaches the command
  // stage; early preemption and preparation must not count a command.
  void NoteCommandBackend(ControllerBackend backend);
  const ControllerCommandCounts& command_counts() const { return command_counts_; }
  void BeginPreparation() { reported_recoveries_ = 0; }
  bool TakeRecoveryNotice(int recoveries) {
    if (recoveries <= reported_recoveries_) {
      return false;
    }
    reported_recoveries_ = recoveries;
    return true;
  }
  void Capture(const NavigationState& state, const Twist2d& measured_velocity);
  const NavigationTelemetryState& state() const { return state_; }

 private:
  NavigationTelemetryState state_;
  ControllerCommandCounts command_counts_;
  int reported_recoveries_ = 0;
};
}  // namespace navigation2d::exploration
