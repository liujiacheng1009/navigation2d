#include "navigation2d/application/exploration/alcove_exit.h"

#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/retreat_execution.h"
#include "navigation2d/application/exploration/route_failure_history.h"
#include "navigation2d/application/exploration/tour_execution.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
void ExecuteAlcoveExit(EntryCoordinator& entry, FrontierExplorer& explorer, TourExecution& tour,
                       RetreatExecution& retreat, RouteFailureHistory& routes,
                       const ExplorationGoal& goal, const AlcoveObservation& observation,
                       AlcoveExitActions& actions) {
  const bool frontier_open = !retreat.active() && explorer.GoalRegionStillFrontier(goal);
  const std::size_t unknown =
      explorer.ReachableUnknownCells(observation.robot.x, observation.robot.y);
  if (frontier_open || unknown > 80) {
    actions.ReportUnfinishedPocket(unknown, frontier_open);
    actions.CancelRecoveryNavigation();
    routes.ClearSignatures();
    if (frontier_open && actions.PassableMouthGoal(goal)) {
      actions.LockEntryMouth(goal);
      actions.PrepareDoorjambRetry();
      if (actions.StartEntryProbe()) {
        return;
      }
    }
    if (!retreat.active()) {
      explorer.RecordAttempt(goal, false);
    }
    if (!tour.goals().empty()) {
      tour.Advance();
    }
    retreat.Finish();
    actions.SelectAfterEntry();
    actions.PublishStatus("SELECTING", "local pocket unfinished; retrying without pocket cool");
    return;
  }
  actions.ReportExhaustedPocket(observation);
  actions.CancelRecoveryNavigation();
  if (!retreat.active()) {
    explorer.RecordAttempt(goal, true);
  }
  explorer.CoolRegion(goal.x, goal.y, 1.80, 3000u);
  explorer.CoolRegion(goal.frontier_x, goal.frontier_y, 1.80, 3000u);
  explorer.CoolRegion(observation.robot.x, observation.robot.y, 1.80, 3000u);
  entry.Clear();
  retreat.Finish();
  retreat.AllowRetry();
  tour.Suspend();
  routes.ClearSignatures();
  if (actions.EscapeOrShortRetreat("leaving alcove tip; peel",
                                   "leaving alcove tip; short trail retreat")) {
    return;
  }
  actions.SelectAfterEntry();
  actions.PublishStatus("SELECTING", "alcove tip left; pocket cooled, picking a farther goal");
}
}  // namespace navigation2d::exploration
