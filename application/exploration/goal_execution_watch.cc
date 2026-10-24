#include "navigation2d/application/exploration/goal_execution_watch.h"

#include <cmath>

#include "navigation2d/application/navigation_system.h"
#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
void GoalExecutionWatch::Start(std::int64_t started_ns, std::int64_t progress_ns) {
  started_ns_ = started_ns;
  best_progress_m_ = 0.;
  progress_ns_ = progress_ns;
}
void GoalExecutionWatch::ObserveProgress(double metres, const GoalExecutionClock& clock) {
  if (metres >= best_progress_m_ + .08) {
    best_progress_m_ = metres;
    progress_ns_ = clock.ExecutionNowNs();
  }
}
double GoalExecutionWatch::Elapsed(const GoalExecutionClock& clock) const {
  return static_cast<double>(clock.ExecutionNowNs() - started_ns_) / 1e9;
}
bool GoalExecutionWatch::TimedOut(const GoalExecutionClock& clock) const {
  return Elapsed(clock) > timeout_s_;
}
bool GoalExecutionWatch::ProgressOlderThan(double seconds, const GoalExecutionClock& clock) const {
  return progress_ns_ > 0 &&
         static_cast<double>(clock.ExecutionNowNs() - progress_ns_) / 1e9 > seconds;
}
bool GoalExecutionWatch::FailureDue(const NavigationState& state, bool returning, double front,
                                    bool corridor_pinch, bool forward_open,
                                    const GoalExecutionClock& clock) const {
  return state.status == navigation2d::NavigationStatus::kBlocked ||
         ((!returning || state.phase != navigation2d::NavigationPhase::kDockToGoal) &&
          TimedOut(clock)) ||
         (state.path_progress_m < .05 && state.recoveries >= 3 && Elapsed(clock) > 20.0) ||
         (state.path_progress_m < .05 && state.requested_command.linear < .01 &&
          navigation2d::HitBlocksSwing(front) && Elapsed(clock) > 8.0) ||
         (state.path_progress_m >= .05 && ProgressOlderThan(corridor_pinch ? 5.0 : 10.0, clock) &&
          (navigation2d::HitBlocksSwing(front) || (state.recoveries >= 2 && !forward_open) ||
           (state.requested_command.linear < .05 &&
            std::abs(state.requested_command.angular) > .15 && !forward_open))) ||
         (state.path_progress_m >= .05 &&
          state.collision_monitor_action == navigation2d::CollisionMonitorAction::kStop &&
          state.requested_command.linear > .01 && state.minimum_ttc_s < .15 &&
          navigation2d::HitInsideBody(front) && ProgressOlderThan(2.0, clock));
}
}  // namespace navigation2d::exploration
