#include "navigation2d/application/exploration/goal_execution.h"
#include "navigation2d/application/exploration/goal_tracking_actions.h"
#include "navigation2d/application/exploration/motion_output.h"
#include "navigation2d/application/exploration/navigation_session.h"
#include "navigation2d/application/exploration/viewpoint_selection.h"
namespace navigation2d::exploration {
bool GoalExecution::TickIdle(NavigationSession& navigation, ViewpointSelection& selection,
                             MotionOutput& motion, GoalTrackingActions& actions,
                             const GoalExecutionClock& clock) {
  if (Returning()) {
    if (!navigation.has_session()) {
      actions.TickIdleReturn();
      return true;
    }
  } else if (!navigation.has_session()) {
    // A no-motion crawl spans ticks after the route was discarded. Give it
    // priority over the idle stop, rather than ending it after one frame.
    if (HasActiveEntryCrawl(entry_) && actions.TourRobot()) {
      ExecuteEntryCrawl(entry_, explorer_, actions.ActiveGoal(), *actions.TourRobot(), actions);
      return true;
    }
    // Drop the last approach command while planning the next viewpoint so the
    // chassis cannot roll from an arrived pose into the planning circle.
    motion.Send({});
    if (selection.Due(clock.ExecutionNowNs())) {
      selection.ScheduleAt(actions.SelectionDeadlineNs(1.));
      actions.SelectNextGoal();
    }
    return true;
  }
  return false;
}
}  // namespace navigation2d::exploration
