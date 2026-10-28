#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/goal_execution.h"
#include "navigation2d/application/exploration/goal_tracking_actions.h"
#include "navigation2d/application/exploration/motion_output.h"
#include "navigation2d/application/exploration/navigation_session.h"
#include "navigation2d/application/exploration/pose_motion_watch.h"
#include "navigation2d/application/exploration/recovery_policy.h"
#include "navigation2d/application/exploration/return_coordinator.h"

namespace navigation2d::exploration {
bool GoalExecution::WatchImmobility(const NavigationState& state, NavigationSession& navigation,
                                    PoseMotionWatch& motion, MotionOutput& output,
                                    GoalTrackingActions& actions, const GoalExecutionClock& clock) {
  const auto observed = actions.TourRobot();
  if (observed) {
    motion.Observe(observed->x, observed->y, clock.ExecutionNowNs());
  }
  if (!navigation.has_session() || !observed) {
    return false;
  }
  const bool entry_drive = entry_.pending() || entry_.locked();
  const auto body = actions.ReadBodyRanges();
  const double clearance = actions.ActivePathMargin(state.global_planning.path_min_clearance_m);
  const bool tip_wedged =
      !entry_drive && actions.IsPinchGeometryStall(clearance, body.front, body.side);
  if (!motion.Stalled({clock.ExecutionNowNs(), watch_.started_ns(), watch_.progress_ns(),
                       state.path_progress_m, entry_drive, tip_wedged,
                       AxisSwingClear(body.front)})) {
    return false;
  }
  const int recoveries = motion.NoteImmobility();
  actions.ReportPoseImmobility({recoveries, returning_.returning(), entry_.pending().has_value(),
                                tip_wedged, state.path_progress_m, body.front});
  output.Send({});
  navigation.Cancel();
  motion.Debounce(clock.ExecutionNowNs());
  // The single-writer caller retains its pose across this synchronous stage.
  // Read after cancellation, at the original recovery-dispatch boundary.
  const auto robot = actions.TourRobot();
  RecoverImmobility(motion, {robot->x, robot->y, state.path_progress_m, entry_drive, tip_wedged},
                    actions);
  return true;
}
}  // namespace navigation2d::exploration
