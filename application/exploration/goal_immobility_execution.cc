#include "navigation2d/application/exploration/goal_failure_execution.h"
#include "navigation2d/application/exploration/pose_motion_watch.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
void GoalFailureExecution::RecoverImmobility(PoseMotionWatch& motion,
                                             ImmobilityRecoveryObservation observation,
                                             GoalFailureActions& actions) {
  if (Returning()) {
    actions.RecoverStalledReturn();
    return;
  }
  if (observation.entry_drive) {
    ResolveImmobileEntry(entry_, explorer_,
                         {observation.robot_x, observation.robot_y, observation.path_progress_m},
                         actions);
    return;
  }
  // Open-front stalls must not consume peel attempts or burn the tour.
  if (!observation.tip_wedged) {
    actions.CoolActiveRecoveryRoute();
    if (motion.TryOpenReplan([&] { return actions.NavigateImmobileGoal(); })) {
      actions.PublishStatus("NAVIGATING", "immobility open-front; replanned goal");
      return;
    }
    motion.ResetOpenReplans();
    actions.DelayRecoverySelection();
    actions.PublishStatus("SELECTING", "immobility open-front; reselect without peel");
    return;
  }
  explorer_.RecordAttempt(actions.ActiveGoal(), false);
  actions.DemoteViewpointForSeconds(actions.ActiveGoal(), 6.);
  explorer_.CoolRegion(actions.ActiveGoal().x, actions.ActiveGoal().y, .80, 1800u);
  explorer_.CoolRegion(actions.ActiveGoal().frontier_x, actions.ActiveGoal().frontier_y, .80,
                       1800u);
  entry_.Clear();
  retreat_.AllowRetry();
  actions.RequestImmobilityReplay();
  actions.DelayRecoverySelection();
  if (actions.EscapeOrShortRetreat("immobility at wall tip; peel out of inflation",
                                   "immobility at wall tip; short trail retreat")) {
    return;
  }
  if (actions.TryRescueDistantMouths("immobility tip; rescue distant mouths")) {
    return;
  }
  actions.DemoteActiveGoal("immobility tip thrash; demoting cooled viewpoint");
}
}  // namespace navigation2d::exploration
