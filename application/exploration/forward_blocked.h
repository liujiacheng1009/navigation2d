#pragma once
#include <string>

#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/recovery_session.h"
#include "navigation2d/application/exploration/retreat_execution.h"
#include "navigation2d/application/exploration/tour_execution.h"
namespace navigation2d::exploration {
// Synchronous effects for a close forward STOP. The executor retains no
// references; selection time and escape sensing are queried by their effects.
class ForwardBlockedActions {
 public:
  virtual ~ForwardBlockedActions() = default;
  virtual void CancelRecoveryNavigation() = 0;
  virtual void SelectAfterEntry() = 0;
  virtual void PublishStatus(const std::string&, const std::string&) = 0;
  virtual void AbandonLockedEntryMouth(const std::string&) = 0;
  virtual bool EscapeOrShortRetreat(const char*, const char*) = 0;
  virtual bool TryStartEscape() = 0;
  virtual void DemoteViewpointForSeconds(const ExplorationGoal&, double seconds) = 0;
};
// Caller has classified an exploring monitor STOP with a close forward hit.
// Locked/pending entry keeps its budget and tour. Ordinary viewpoints consume
// a failure, cool for eight seconds and advance before attempting escape.
void ExecuteForwardBlocked(EntryCoordinator&, FrontierExplorer&, TourExecution&,
                           GoalRecoverySession&, RetreatExecution&,
                           const ExplorationGoal& active_goal, ForwardBlockedActions&);
}  // namespace navigation2d::exploration
