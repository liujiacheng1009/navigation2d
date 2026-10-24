#include "navigation2d/application/exploration/entry_selection.h"

#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
EntrySelectionResult SelectLockedEntry(EntryCoordinator& entry, const FrontierExplorer& explorer,
                                       const std::optional<TourPose>& robot,
                                       const ExplorationGoal& active_goal,
                                       EntrySelectionActions& actions) {
  // A locked doorway owns selection until crossed or hard-failed.
  if (entry.locked() && !entry.pending()) {
    entry.SetPending(entry.locked());
  }
  if (!entry.pending()) {
    return EntrySelectionResult::kContinueTour;
  }
  if (robot && entry.Satisfied(robot->x, robot->y, active_goal, explorer)) {
    actions.AcknowledgeEntry(EntryArrival::kSelection);
    return EntrySelectionResult::kHandled;
  }
  const int before_attempts = entry.attempts();
  if (before_attempts < entry.AttemptBudget(explorer) && actions.StartEntryProbe()) {
    return EntrySelectionResult::kHandled;
  }
  if (entry.EasyNow(explorer) && entry.attempts() < entry.AttemptBudget(explorer)) {
    if (entry.crawl_ticks() == 0) {
      entry.StartCrawl();
    }
    actions.DeferEntrySelection(EntrySelectionDelay::kWideMouthCrawl);
    return EntrySelectionResult::kHandled;
  }
  if (entry.attempts() > before_attempts && entry.attempts() < entry.AttemptBudget(explorer)) {
    actions.DeferEntrySelection(EntrySelectionDelay::kRejectedPlan);
    return EntrySelectionResult::kHandled;
  }
  // Continue the committed tour here; retreat/re-approach made hallway loops.
  actions.AbandonLockedEntryMouth("entry blocked; leave mouth and continue tour (no reverse)");
  return EntrySelectionResult::kContinueTour;
}
}  // namespace navigation2d::exploration
