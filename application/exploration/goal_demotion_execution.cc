#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/exploration_mission.h"
#include "navigation2d/application/exploration/goal_execution.h"
#include "navigation2d/application/exploration/goal_tracking_actions.h"
#include "navigation2d/application/exploration/navigation_session.h"
#include "navigation2d/application/exploration/recovery_session.h"
#include "navigation2d/application/exploration/return_coordinator.h"
#include "navigation2d/application/exploration/tour_execution.h"
#include "navigation2d/application/exploration/viewpoint_selection.h"
#include "navigation2d/exploration/frontier_explorer.h"

namespace navigation2d::exploration {
void GoalExecution::DemoteActive(NavigationSession& navigation, ViewpointSelection& selection,
                                 const char* reason, GoalTrackingActions& actions) {
  if (mission_.phase() != MissionPhase::kReturning && !returning_.returning()) {
    explorer_.RecordAttempt(entry_.pending().value_or(actions.ActiveGoal()), false);
    actions.DemoteViewpointForSeconds(entry_.pending().value_or(actions.ActiveGoal()), 8.);
  }
  entry_.Clear();
  navigation.Cancel();
  actions.ClearRecoveryDisplay();
  recovery_.Clear();
  if (!tour_.goals().empty()) {
    tour_.Advance();
  }
  // Dwell before hopping mouths; rebuilding immediately causes junction thrash.
  selection.ScheduleAt(actions.SelectionDeadlineNs(1.5));
  actions.PublishStatus("SELECTING", reason);
}
}  // namespace navigation2d::exploration
