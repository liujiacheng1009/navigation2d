#include <cmath>

#include "navigation2d/application/exploration/goal_failure_execution.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
void GoalFailureExecution::LeavePinch(const char* reason, GoalFailureActions& actions) {
  actions.CoolActiveRecoveryRoute();
  actions.ReportPinchRecovery(reason);
  actions.StopSelectionMotion();
  if (!Returning() && std::isfinite(actions.ActiveGoal().x) &&
      std::isfinite(actions.ActiveGoal().y)) {
    explorer_.RecordAttempt(entry_.pending().value_or(actions.ActiveGoal()), false);
    // A short cooldown leaves other mouths available during a bounded peel.
    actions.DemoteViewpointForSeconds(entry_.pending().value_or(actions.ActiveGoal()), 6.);
    explorer_.CoolRegion(actions.ActiveGoal().x, actions.ActiveGoal().y, .70, 1500u);
    explorer_.CoolRegion(actions.ActiveGoal().frontier_x, actions.ActiveGoal().frontier_y, .70,
                         1500u);
  }
  entry_.Clear();
  actions.CancelRecoveryNavigation();
  actions.ClearRecoveryDisplay();
  if (!tour_.goals().empty()) {
    tour_.Advance();
  }
  recovery_.Clear();
  retreat_.AllowRetry();
  if (actions.EscapeOrShortRetreat("pinch unstick; peel into free",
                                   "pinch unstick; short trail retreat")) {
    actions.PublishStatus("RECOVERING", reason);
    return;
  }
  // Do not spend idle completing rounds in the tip if a distant mouth is viable.
  if (actions.TryRescueDistantMouths(reason)) {
    return;
  }
  actions.DelayRecoverySelection();
  actions.PublishStatus("SELECTING", reason);
}
}  // namespace navigation2d::exploration
