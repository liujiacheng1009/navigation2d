#include "navigation2d/application/exploration/goal_execution.h"
#include "navigation2d/application/exploration/goal_tracking_actions.h"
#include "navigation2d/application/exploration/pose_motion_watch.h"
#include "navigation2d/application/exploration/viewpoint_rejections.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
void GoalExecution::CommitAccepted(AcceptedNavigation accepted, PoseMotionWatch& motion,
                                   ViewpointRejections& rejections, GoalTrackingActions& actions,
                                   const GoalExecutionClock& clock) {
  const auto& goal = accepted.goal;
  returning_.AcceptNavigationMode(accepted.returning);
  // Resume/latest-map replan of the same viewpoint retains peel/scrape budgets.
  recovery_.Begin(goal.x, goal.y);
  const auto started_ns = clock.ExecutionNowNs();
  const auto progress_ns = clock.ExecutionNowNs();
  watch_.Start(started_ns, progress_ns);
  motion.ResetOpenReplans();
  rejections.ResetScrapes();
  actions.PublishStatus(
      accepted.returning ? "RETURNING" : (goal.leftover_approach ? "COMPLETING" : "NAVIGATING"),
      accepted.returning
          ? "returning to exploration start"
          : (retreat_.active()
                 ? "leaving a pinched corridor along the driven trail"
                 : (goal.leftover_approach ? "one leftover approach after exploring tour emptied"
                                           : "navigating to frontier")));
  // Status publication precedes the DFS commitment. A synchronous callback may
  // alter retreat state, so read it live again at the original commit boundary.
  if (!accepted.returning && !retreat_.active()) {
    explorer_.CommitRegion(goal);
  }
  actions.ReportAcceptedNavigation(goal, accepted.initial_state);
}
}  // namespace navigation2d::exploration
