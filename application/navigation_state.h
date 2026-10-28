#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

#include "navigation2d/control/collision_monitor.h"
#include "navigation2d/control/local_controller.h"
#include "navigation2d/geometry/twist_2d.h"
#include "navigation2d/planning/global_planner.h"
namespace navigation2d {
enum class NavigationStatus { kIdle, kNavigating, kSucceeded, kBlocked };
enum class NavigationPhase { kAlignToPath, kTrackPath, kDockToGoal };

struct NavigationState {
  NavigationStatus status = NavigationStatus::kIdle;
  GlobalPlanStatus planning_status = GlobalPlanStatus::kNotRun;
  LocalControlStatus control_status = LocalControlStatus::kNotRun;
  AlgorithmStop algorithm_stop = AlgorithmStop::kNone;
  NavigationPhase phase = NavigationPhase::kAlignToPath;
  Twist2d command;
  int replans = 0;
  int emergency_stops = 0;
  int recoveries = 0;
  double global_path_length_m = 0.;
  // Route-coordinate telemetry for recovery decisions.  `path_progress_m`
  // is monotonic within one global plan; the two flags identify whether a
  // zero command came from the tracker itself or from live safety filtering.
  double path_progress_m = 0.;
  bool controller_commanded_motion = false;
  bool safety_stopped_motion = false;
  // Persist the last actually evaluated control cycle across an early
  // progress-recovery return. This distinguishes a tracker zero from a
  // safety filter veto; these values are reset only for a new goal.
  Twist2d requested_command;
  Twist2d published_command;
  // Duration of the latest Compute/ComputeRequest hook. Excludes request
  // validation, separate Diagnostics() access, sensing, planning and safety.
  double controller_solve_us = 0.;
  ControllerDiagnostics controller_diagnostics;
  // Preserve the planner's concrete rejection reason for ROS diagnostics;
  // callers should not have to infer start collision vs. graph disconnect
  // from a generic zero-length path.
  std::string planning_failure_reason;
  CollisionMonitorAction collision_monitor_action = CollisionMonitorAction::kNone;
  double minimum_ttc_s = 0.;
  GlobalPlanningDiagnostics global_planning;
  int obstacle_heuristic_cache_hits = 0;
  int incremental_replans = 0;
  std::size_t incremental_repaired_states = 0;
  std::uint64_t costmap_digest = 0;
  // First-segment identity: pose cell, start yaw bin and the first ~1 m of
  // the committed route.  Callers cool a failed signature so the next plan
  // cannot replay the same in-place alignment.
  std::uint64_t path_signature = 0;
};

}  // namespace navigation2d
