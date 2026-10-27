#include "navigation2d/application/exploration/forward_blocked.h"

#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
void ExecuteForwardBlocked(EntryCoordinator& entry, FrontierExplorer& explorer, TourExecution& tour,
                           GoalRecoverySession& recovery, RetreatExecution& retreat,
                           const ExplorationGoal& active_goal, ForwardBlockedActions& actions) {
  if (entry.locked() || entry.pending()) {
    if (!entry.pending()) {
      entry.SetPending(entry.locked());
    }
    retreat.AllowRetry();
    actions.CancelRecoveryNavigation();
    if (actions.EscapeOrShortRetreat("forward blocked at locked mouth; peel", "unused")) {
      return;
    }
    actions.AbandonLockedEntryMouth(
        "forward blocked at mouth; leave and continue tour (no reverse)");
    actions.SelectAfterEntry();
    return;
  }
  explorer.RecordAttempt(entry.pending().value_or(active_goal), false);
  entry.Clear();
  actions.DemoteViewpointForSeconds(active_goal, 8.0);
  if (!tour.goals().empty()) {
    tour.Advance();
  }
  recovery.Clear();
  if (actions.TryStartEscape()) {
    actions.CancelRecoveryNavigation();
    actions.PublishStatus("RECOVERING", "forward blocked; escape then advance tour");
    return;
  }
  actions.CancelRecoveryNavigation();
  actions.SelectAfterEntry();
  actions.PublishStatus("SELECTING", "forward blocked; skipping viewpoint");
}
}  // namespace navigation2d::exploration
