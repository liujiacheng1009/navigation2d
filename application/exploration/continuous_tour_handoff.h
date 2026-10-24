#pragma once
#include "navigation2d/application/exploration/navigation_session.h"
#include "navigation2d/application/exploration/tour_execution.h"
namespace navigation2d::exploration {
class ContinuousHandoffActions {
 public:
  virtual ~ContinuousHandoffActions() = default;
  virtual bool NavigateHandoff(const ExplorationGoal&) = 0;
  virtual void HandoffAccepted() = 0;
};
// Requires an active session and >=2 committed tasks. Preparation replaces
// navigation synchronously; rejection restores the original owned session.
// On acceptance the previous session stays alive through bookkeeping/reporting.
bool TryContinuousTourHandoff(TourExecution&, FrontierExplorer&, NavigationSession&,
                              const ExplorationGoal& active_goal, TourPose robot,
                              ContinuousHandoffActions&);
}  // namespace navigation2d::exploration
