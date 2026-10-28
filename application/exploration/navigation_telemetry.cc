#include "navigation2d/application/exploration/navigation_telemetry.h"
namespace navigation2d::exploration {
namespace {
const char* BackendName(navigation2d::ControllerBackend backend) {
  switch (backend) {
    case navigation2d::ControllerBackend::kAcados:
      return "acados";
    case navigation2d::ControllerBackend::kMppi:
      return "mppi";
    case navigation2d::ControllerBackend::kRpp:
      return "rpp";
    case navigation2d::ControllerBackend::kDwa:
      return "dwa";
    case navigation2d::ControllerBackend::kNone:
      return "none";
  }
  return "unknown";
}

const char* NavStatusName(navigation2d::NavigationStatus status) {
  switch (status) {
    case navigation2d::NavigationStatus::kIdle:
      return "idle";
    case navigation2d::NavigationStatus::kNavigating:
      return "navigating";
    case navigation2d::NavigationStatus::kSucceeded:
      return "succeeded";
    case navigation2d::NavigationStatus::kBlocked:
      return "blocked";
  }
  return "unknown";
}

const char* NavPhaseName(navigation2d::NavigationPhase phase) {
  switch (phase) {
    case navigation2d::NavigationPhase::kAlignToPath:
      return "align_to_path";
    case navigation2d::NavigationPhase::kTrackPath:
      return "track_path";
    case navigation2d::NavigationPhase::kDockToGoal:
      return "dock_to_goal";
  }
  return "unknown";
}

const char* ManeuverName(navigation2d::ControllerManeuver maneuver) {
  switch (maneuver) {
    case navigation2d::ControllerManeuver::kTracking:
      return "tracking";
    case navigation2d::ControllerManeuver::kStopping:
      return "stopping";
    case navigation2d::ControllerManeuver::kRotateToPath:
      return "rotate_to_path";
    case navigation2d::ControllerManeuver::kRouteInfeasible:
      return "route_infeasible";
  }
  return "unknown";
}

const char* MonitorActionName(navigation2d::CollisionMonitorAction action) {
  switch (action) {
    case navigation2d::CollisionMonitorAction::kNone:
      return "none";
    case navigation2d::CollisionMonitorAction::kSlowdown:
      return "slowdown";
    case navigation2d::CollisionMonitorAction::kStop:
      return "stop";
    case navigation2d::CollisionMonitorAction::kBlindZoneStop:
      return "blind_zone_stop";
    case navigation2d::CollisionMonitorAction::kSourceTimeout:
      return "source_timeout";
  }
  return "unknown";
}

}  // namespace
void NavigationTelemetry::NoteCommandBackend(ControllerBackend backend) {
  switch (backend) {
    case ControllerBackend::kAcados: ++command_counts_.acados; break;
    case ControllerBackend::kMppi: ++command_counts_.mppi; break;
    case ControllerBackend::kRpp: ++command_counts_.rpp; break;
    case ControllerBackend::kDwa:
    case ControllerBackend::kNone: break;
  }
}
void NavigationTelemetry::Capture(const navigation2d::NavigationState& state,
                                  const navigation2d::Twist2d& measured_velocity) {
  state_.backend = BackendName(state.controller_diagnostics.backend);
  state_.nav_status = NavStatusName(state.status);
  state_.nav_phase = NavPhaseName(state.phase);
  state_.maneuver = ManeuverName(state.controller_diagnostics.maneuver);
  state_.monitor_action = MonitorActionName(state.collision_monitor_action);
  state_.path_progress_m = state.path_progress_m;
  state_.path_length_m = state.global_path_length_m;
  state_.path_max_curvature = state.global_planning.path_max_curvature;
  state_.replans = state.replans;
  state_.incremental_replans = state.incremental_replans;
  state_.recoveries = state.recoveries;
  state_.emergency_stops = state.emergency_stops;
  state_.cmd_linear = state.command.linear;
  state_.cmd_angular = state.command.angular;
  state_.requested_linear = state.requested_command.linear;
  state_.requested_angular = state.requested_command.angular;
  state_.published_linear = state.published_command.linear;
  state_.published_angular = state.published_command.angular;
  state_.measured_linear = measured_velocity.linear;
  state_.measured_angular = measured_velocity.angular;
  state_.controller_commanded = state.controller_commanded_motion;
  state_.safety_stopped = state.safety_stopped_motion;
  state_.intentional_stop = state.controller_diagnostics.intentional_stop;
  state_.fallback_level = state.controller_diagnostics.fallback_level;
  state_.controller_solve_us = state.controller_diagnostics.solve_us > 0.
                                   ? state.controller_diagnostics.solve_us
                                   : state.controller_solve_us;
  state_.minimum_ttc_s = state.minimum_ttc_s;
  state_.planner_expansions = static_cast<int>(state.global_planning.expansions);
  state_.planner_elapsed_s = state.global_planning.elapsed_s;
}

}  // namespace navigation2d::exploration
