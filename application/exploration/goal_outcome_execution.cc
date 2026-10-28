#include <cmath>
#include <limits>

#include "navigation2d/application/exploration/exploration_mission.h"
#include "navigation2d/application/exploration/goal_execution.h"
#include "navigation2d/application/exploration/goal_tracking_actions.h"
#include "navigation2d/application/exploration/navigation_session.h"
#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
void GoalExecution::FinishSucceeded(NavigationSession& navigation, ViewpointArrival& arrival,
                                    GoalTrackingActions& actions) {
  navigation.Cancel();
  if (Returning()) {
    actions.CompleteReturnNavigation();
  } else if (mission_.phase() == MissionPhase::kCompleting) {
    explorer_.RecordAttempt(actions.ActiveGoal(), true);
    actions.FinishCompleting();
  } else if (retreat_.active()) {
    retreat_.Finish();
    actions.CompleteRetreatNavigation();
  } else {
    arrival.Succeeded(explorer_, entry_, tour_, actions.ActiveGoal(), *actions.TourRobot(),
                      actions);
  }
}
void GoalExecution::FinishCycle(const NavigationState& state, NavigationSession& navigation,
                                ViewpointArrival& arrival, RouteFailureHistory& routes,
                                GoalTrackingActions& actions, const GoalExecutionClock& clock) {
  const auto robot = actions.TourRobot();
  const double distance =
      robot ? std::hypot(actions.ActiveGoal().x - robot->x, actions.ActiveGoal().y - robot->y)
            : std::numeric_limits<double>::infinity();
  const auto body = actions.ReadBodyRanges();
  const bool forward_open = AxisSwingClear(body.front);
  const bool pinch = HitBlocksSwing(body.front) && HitBlocksSwing(body.side);
  // Keep alcove exit before success handling, including active trail retreats.
  const bool alcove = mission_.phase() == MissionPhase::kExploring && !returning_.returning() &&
                      (distance < kSwingClearance() ||
                       (state.global_path_length_m > .20 &&
                        state.path_progress_m >= .80 * state.global_path_length_m)) &&
                      watch_.ProgressOlderThan(2.5, clock) &&
                      state.requested_command.linear < .06 && pinch;
  if (alcove) {
    ExecuteAlcoveExit(entry_, explorer_, tour_, retreat_, routes, actions.ActiveGoal(),
                      {*actions.TourRobot(), distance, body.front, body.side, state.path_progress_m,
                       state.global_path_length_m},
                      actions);
    return;
  }
  if (state.status == NavigationStatus::kSucceeded) {
    FinishSucceeded(navigation, arrival, actions);
  } else if (watch_.FailureDue(state, Returning(), body.front, pinch, forward_open, clock)) {
    Resolve(state, body.front, body.side, clock, actions);
  }
}
}  // namespace navigation2d::exploration
