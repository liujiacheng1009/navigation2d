#pragma once
#include <limits>

#include "navigation2d/application/command_stall_watch.h"
#include "navigation2d/application/navigation_failure.h"
namespace navigation2d {
struct ExecutionWatchdogSettings {
  double period_s, progress_radius_m, progress_timeout_s;
  int maximum_recovery_attempts;
};
struct PreviousCommandObservation {
  Twist2d command;
  double path_progress_m, timestamp_s;
  bool safety_stopped, aligning;
};
enum class StallMode { kDocking, kAlignment, kTracking };
enum class RotationStage { kAlignment, kBeforeController, kAfterController };
// Owns temporal progress, rotation, stop and per-goal recovery budgets. It
// consumes facts and returns failures; it never clears paths or edits NavState.
// Goal reset, route commit and planning failure retain distinct reset scopes.
class ExecutionWatchdog {
 public:
  explicit ExecutionWatchdog(ExecutionWatchdogSettings settings)
      : settings_(settings), stall_(settings.period_s) {}
  void NewGoal();
  void NewRoute();
  void PlanningFailed() { stall_.Reset(); }
  void PlanningAttempted(double stamp_s) { last_progress_s_ = stamp_s; }
  void BeginCycle(double stamp_s);
  double path_anchor_m() const { return path_anchor_m_; }
  void NoteDocking(bool distance_progress, bool yaw_progress, double stamp_s);
  bool ObserveAlignment(double score, double heading_error_rad, double path_progress_m,
                        double stamp_s);
  NavigationFailure ObserveSafetyStop(bool stopped, double stamp_s);
  NavigationFailure ObservePrevious(const PreviousCommandObservation& observation);
  NavigationFailure ObserveStall(const StallObservation& observation, StallMode mode);
  NavigationFailure ObserveController(bool commanded_motion, bool safety_stopped,
                                      bool intentional_stop);
  void ChargeRotation(double angular_rps);
  void ChargeController(Twist2d command);
  NavigationFailure CheckRotation(RotationStage stage);
  NavigationFailure DockingTimeout(double stamp_s) const;

 private:
  ExecutionWatchdogSettings settings_;
  CommandStallWatch stall_;
  double last_progress_s_ = 0., path_anchor_m_ = 0.;
  double rotation_rad_ = 0., safety_stop_since_s_ = -1.;
  double best_alignment_score_ = std::numeric_limits<double>::infinity();
  bool initialized_ = false;
  int attempts_ = 0, controller_blocked_cycles_ = 0;
};
}  // namespace navigation2d
