#include "navigation2d/application/execution_watchdog.h"

#include <cmath>
namespace navigation2d {
void ExecutionWatchdog::NewGoal() {
  // Goal entry restarts only the command-cycle budget. A successful route
  // commit separately resets the encoder/pose window and sensor-stop timer.
  initialized_ = false;
  path_anchor_m_ = 0.;
  attempts_ = 0;
  controller_blocked_cycles_ = 0;
  rotation_rad_ = 0.;
  best_alignment_score_ = std::numeric_limits<double>::infinity();
  stall_.RestartCycleBudget();
}
void ExecutionWatchdog::NewRoute() {
  // Route replacement does not renew the per-goal recovery allowance.
  path_anchor_m_ = 0.;
  best_alignment_score_ = std::numeric_limits<double>::infinity();
  controller_blocked_cycles_ = 0;
  rotation_rad_ = 0.;
  safety_stop_since_s_ = -1.;
  stall_.Reset();
}
void ExecutionWatchdog::BeginCycle(double stamp_s) {
  if (!initialized_) {
    last_progress_s_ = stamp_s;
    initialized_ = true;
  }
}
void ExecutionWatchdog::NoteDocking(bool distance_progress, bool yaw_progress, double stamp_s) {
  if (distance_progress) {
    attempts_ = 0;
  }
  if (distance_progress || yaw_progress) {
    last_progress_s_ = stamp_s;
  }
}
bool ExecutionWatchdog::ObserveAlignment(double score, double heading, double progress,
                                         double stamp_s) {
  if (score + .02 < best_alignment_score_) {
    best_alignment_score_ = score;
    last_progress_s_ = stamp_s;
  }
  if (std::abs(heading) < .25 || progress >= settings_.progress_radius_m) {
    last_progress_s_ = stamp_s;
    return true;
  }
  return false;
}
NavigationFailure ExecutionWatchdog::ObserveSafetyStop(bool stopped, double stamp_s) {
  // A filtered zero must not refresh progress forever. Two seconds rejects
  // one-cycle latches while surfacing blockage before the mission timeout.
  if (stopped) {
    if (safety_stop_since_s_ < 0.) {
      safety_stop_since_s_ = stamp_s;
    }
    if (stamp_s - safety_stop_since_s_ >= 2.) {
      ++attempts_;
      safety_stop_since_s_ = -1.;
      return NavigationFailure::kCollisionHold;
    }
  } else {
    safety_stop_since_s_ = -1.;
  }
  return NavigationFailure::kNone;
}
NavigationFailure ExecutionWatchdog::ObservePrevious(const PreviousCommandObservation& o) {
  const bool rotating = std::abs(o.command.linear) <= 1e-3 && std::abs(o.command.angular) > 1e-3;
  const bool stopped = std::abs(o.command.linear) <= 1e-3 && std::abs(o.command.angular) <= 1e-3;
  // Rotation suspends linear progress only while paying its full-turn budget.
  // An intentional zero suspends it; a safety zero keeps the clock running.
  if (rotating) {
    ChargeRotation(o.command.angular);
    const auto failure = CheckRotation(RotationStage::kBeforeController);
    if (failure != NavigationFailure::kNone) {
      return failure;
    }
    last_progress_s_ = o.timestamp_s;
  } else if (stopped && !o.safety_stopped) {
    last_progress_s_ = o.timestamp_s;
  } else if (o.path_progress_m >= path_anchor_m_ + settings_.progress_radius_m) {
    path_anchor_m_ = o.path_progress_m;
    last_progress_s_ = o.timestamp_s;
    attempts_ = 0;
    rotation_rad_ = 0.;
  } else if (o.timestamp_s - last_progress_s_ >
             settings_.progress_timeout_s * (o.aligning ? 3. : 1.)) {
    ++attempts_;
    last_progress_s_ = o.timestamp_s;
    return attempts_ > settings_.maximum_recovery_attempts ? NavigationFailure::kProgressExhausted
                                                           : NavigationFailure::kProgressRetry;
  }
  return NavigationFailure::kNone;
}
NavigationFailure ExecutionWatchdog::ObserveStall(const StallObservation& observation,
                                                  StallMode mode) {
  if (!stall_.Observe(observation)) {
    return NavigationFailure::kNone;
  }
  ++attempts_;
  if (mode != StallMode::kAlignment) {
    stall_.RestartCycleBudget();
  }
  return mode == StallMode::kDocking ? NavigationFailure::kDockingNoMotion
                                     : NavigationFailure::kControllerNoMotion;
}
NavigationFailure ExecutionWatchdog::ObserveController(bool commanded, bool safety_stopped,
                                                       bool intentional_stop) {
  if (!commanded && !safety_stopped && !intentional_stop) {
    if (++controller_blocked_cycles_ >= 3) {
      ++attempts_;
      controller_blocked_cycles_ = 0;
      return attempts_ > settings_.maximum_recovery_attempts
                 ? NavigationFailure::kControllerExhausted
                 : NavigationFailure::kControllerRetry;
    }
  } else {
    controller_blocked_cycles_ = 0;
  }
  return NavigationFailure::kNone;
}
void ExecutionWatchdog::ChargeRotation(double angular_rps) {
  rotation_rad_ += std::abs(angular_rps) * settings_.period_s;
}
void ExecutionWatchdog::ChargeController(Twist2d command) {
  if (std::abs(command.linear) <= 1e-4 && std::abs(command.angular) > 1e-4) {
    ChargeRotation(command.angular);
  }
}
NavigationFailure ExecutionWatchdog::CheckRotation(RotationStage stage) {
  if (!(rotation_rad_ >= 2. * std::acos(-1.))) {
    return NavigationFailure::kNone;
  }
  ++attempts_;
  rotation_rad_ = 0.;
  if (stage == RotationStage::kAlignment) {
    return NavigationFailure::kAlignmentFullTurn;
  }
  return stage == RotationStage::kBeforeController ? NavigationFailure::kTrackingFullTurn
                                                   : NavigationFailure::kPublishedFullTurn;
}
NavigationFailure ExecutionWatchdog::DockingTimeout(double stamp_s) const {
  return stamp_s - last_progress_s_ > settings_.progress_timeout_s * 3.
             ? NavigationFailure::kDockingTimeout
             : NavigationFailure::kNone;
}
}  // namespace navigation2d
