#pragma once
#include <string>

#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/recovery_session.h"
namespace navigation2d::exploration {
class EntryRecoveryActions {
 public:
  virtual ~EntryRecoveryActions() = default;
  virtual bool StartEntryProbe() = 0;
  virtual void AbandonLockedEntryMouth(const std::string&) = 0;
  virtual void PublishStatus(const std::string& phase, const std::string& message) = 0;
  virtual void SelectAfterEntry() = 0;
  virtual void CancelRecoveryNavigation() = 0;
  virtual void PrepareDoorjambRetry() = 0;
};
// Synchronous continuation policies, borrowing the single entry-state owner.
// Peel/doorjamb/stall require an active mouth; callbacks may update budgets or
// clear it. A retreat resumes only a lock, even if another pending mouth exists.
void ResumeLockedEntryAfterPeel(EntryCoordinator&, const FrontierExplorer&, EntryRecoveryActions&);
void RetryLockedEntryAfterDoorjamb(EntryCoordinator&, const FrontierExplorer&,
                                   EntryRecoveryActions&);
[[nodiscard]] bool ResumeEntryAfterRetreat(EntryCoordinator&, const FrontierExplorer&,
                                           EntryRecoveryActions&);
// Caller already cancelled navigation and handled the no-motion crawl branch.
void ResolveLockedEntryStall(EntryCoordinator&, const FrontierExplorer&, GoalRecoverySession&,
                             EntryRecoveryActions&);
}  // namespace navigation2d::exploration
