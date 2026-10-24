#include "navigation2d/application/exploration/entry_recovery_execution.h"

#include <cassert>

#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
namespace {
void RestorePending(EntryCoordinator& entry) {
  assert(entry.pending() || entry.locked());
  if (!entry.pending()) {
    entry.SetPending(entry.locked());
  }
}
}  // namespace
void ResumeLockedEntryAfterPeel(EntryCoordinator& entry, const FrontierExplorer& explorer,
                                EntryRecoveryActions& actions) {
  RestorePending(entry);
  if (entry.attempts() < entry.AttemptBudget(explorer) && actions.StartEntryProbe()) {
    actions.PublishStatus("COMPLETING", "resumed locked mouth after peel");
    return;
  }
  if (entry.EasyNow(explorer)) {
    if (entry.crawl_ticks() == 0) {
      entry.StartCrawl();
    }
    actions.PublishStatus("COMPLETING", "peel done; wide mouth crawl instead of abandon");
    return;
  }
  actions.AbandonLockedEntryMouth("peel done; leave mouth and continue tour (no reverse)");
  actions.SelectAfterEntry();
}
void RetryLockedEntryAfterDoorjamb(EntryCoordinator& entry, const FrontierExplorer& explorer,
                                   EntryRecoveryActions& actions) {
  RestorePending(entry);
  actions.CancelRecoveryNavigation();
  // Deliberately retain route cooling: clearing it re-enabled a banned chord.
  actions.PrepareDoorjambRetry();
  if (entry.attempts() < entry.AttemptBudget(explorer) && actions.StartEntryProbe()) {
    actions.PublishStatus("COMPLETING", "doorjamb STOP; retrying locked mouth entry");
    return;
  }
  actions.AbandonLockedEntryMouth("doorjamb STOP; leave mouth and continue tour (no reverse)");
  actions.SelectAfterEntry();
}
bool ResumeEntryAfterRetreat(EntryCoordinator& entry, const FrontierExplorer& explorer,
                             EntryRecoveryActions& actions) {
  if (entry.locked() && entry.attempts() < entry.AttemptBudget(explorer)) {
    entry.SetPending(entry.locked());
    if (actions.StartEntryProbe()) {
      return true;
    }
  }
  return false;
}
void ResolveLockedEntryStall(EntryCoordinator& entry, const FrontierExplorer& explorer,
                             GoalRecoverySession& recovery, EntryRecoveryActions& actions) {
  RestorePending(entry);
  if (entry.attempts() < entry.AttemptBudget(explorer) && actions.StartEntryProbe()) {
    return;
  }
  actions.AbandonLockedEntryMouth("entry stall; leave mouth and continue tour (no reverse)");
  recovery.Clear();
  actions.SelectAfterEntry();
}
}  // namespace navigation2d::exploration
