#include "navigation2d/application/exploration/retreat_execution.h"
namespace navigation2d::exploration {
bool RetreatExecution::Start(DrivenTrail trail, TourPose robot, std::pair<int, int> cell,
                             bool entry, RetreatActions& actions) {
  if (trail.size() < 4 || attempted_cell_ == cell) {
    return false;
  }
  const bool pinched = actions.RetreatPinched();
  const auto goal = SelectTrailRetreat(trail, robot, entry, pinched);
  if (!goal) {
    return false;
  }
  actions.ClearRetreatCooling();
  attempted_cell_ = cell;
  actions.PrepareRetreatEvidence(pinched);
  active_ = true;
  if (actions.NavigateRetreat(*goal)) {
    return true;
  }
  active_ = false;
  actions.CancelRetreatEvidence();
  return false;
}
}  // namespace navigation2d::exploration
