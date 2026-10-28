#include "navigation2d/application/exploration/goal_execution.h"
#include "navigation2d/application/exploration/route_failure_history.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
bool GoalExecution::HandleMappedThrash(RouteFailureHistory& routes, GoalFailureActions& actions) {
  if (!routes.TakeMappedThrash()) {
    return false;
  }
  actions.CancelRecoveryNavigation();
  recovery_.Clear();
  routes.ClearSignatures();
  const std::size_t local_unknown = actions.ReachableUnknownForRecovery();
  // An unfinished pocket gets an entry attempt before cooling its region.
  if (local_unknown > 80 && actions.TourRobot()) {
    actions.ReportMappedRejectThrash(local_unknown);
    actions.LockEntryMouth(actions.ActiveGoal());
    actions.PrepareDoorjambRetry();
    if (actions.StartEntryProbe()) {
      return true;
    }
    explorer_.RecordAttempt(actions.ActiveGoal(), false);
    entry_.Clear();
    if (!tour_.goals().empty()) {
      tour_.Advance();
    }
    retreat_.AllowRetry();
    if (actions.EscapeOrShortRetreat(
            "mapped-obstacle thrash; peel then retry local pocket",
            "mapped-obstacle thrash; short retreat then retry local pocket")) {
      return true;
    }
    actions.SelectAfterEntry();
    actions.PublishStatus("SELECTING", "mapped-obstacle thrash; local pocket kept");
    return true;
  }
  actions.ReportMappedRejectThrash(0);
  entry_.Clear();
  if (const auto robot = actions.TourRobot()) {
    explorer_.CoolRegion(actions.ActiveGoal().x, actions.ActiveGoal().y, 1.80, 3000u);
    explorer_.CoolRegion(actions.ActiveGoal().frontier_x, actions.ActiveGoal().frontier_y, 1.80,
                         3000u);
    explorer_.CoolRegion(robot->x, robot->y, 1.80, 3000u);
  }
  retreat_.AllowRetry();
  tour_.Suspend();
  actions.PublishStatus("SELECTING", "leaving pinch after mapped-obstacle rejects");
  if (actions.EscapeOrShortRetreat("leaving pinch after mapped-obstacle rejects; peel",
                                   "leaving pinch after mapped-obstacle rejects; short retreat")) {
    return true;
  }
  actions.DelayMappedRejectSelection();
  return true;
}
}  // namespace navigation2d::exploration
