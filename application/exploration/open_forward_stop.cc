#include "navigation2d/application/exploration/open_forward_stop.h"
namespace navigation2d::exploration {
void ExecuteOpenForwardStop(EntryCoordinator& entry, const FrontierExplorer& explorer,
                            GoalRecoverySession& recovery, const GoalExecutionWatch& watch,
                            const GoalExecutionClock& clock, OpenForwardStopObservation observation,
                            OpenForwardStopActions& actions) {
  if (entry.locked() || entry.pending()) {
    if (!entry.pending()) {
      entry.SetPending(entry.locked());
    }
    if (watch.ProgressOlderThan(4.0, clock)) {
      RetryLockedEntryAfterDoorjamb(entry, explorer, actions);
    }
    return;
  }
  if (actions.OpenForwardWallScrape(observation.path_clearance_m, observation.side_range_m)) {
    actions.BreakWallScrapeThrash("open-forward wall-scrape STOP");
    return;
  }
  if (watch.ProgressOlderThan(6.0, clock)) {
    if (recovery.open_replans < 1 &&
        actions.ReplanActiveGoalOnLatestMap("open-forward side stall; replan without peel")) {
      ++recovery.open_replans;
      return;
    }
    actions.DeferViewpointSelection(.35, "open-forward side stall; reselect without peel");
    actions.CancelRecoveryNavigation();
  }
}
}  // namespace navigation2d::exploration
