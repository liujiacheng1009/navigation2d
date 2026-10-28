#include "navigation2d/application/exploration/completion_execution.h"
#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/exploration_mission.h"
#include "navigation2d/application/exploration/tour_execution.h"
#include "navigation2d/application/exploration/viewpoint_rejections.h"
#include "navigation2d/application/exploration/viewpoint_selection.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
bool ViewpointSelection::RescueDistant(ExplorationMission& mission, bool returning,
                                       const char* reason, ViewpointSelectionActions& actions) {
  if (rescue_attempted_ || returning || mission.phase() == MissionPhase::kReturning ||
      !actions.TourRobot()) {
    return false;
  }
  if (!explorer_.HasExploringWork()) {
    return false;
  }
  rescue_attempted_ = true;
  actions.ReportSelectionRescue(reason, false);
  viewpoint_rejections_.ClearDemotion();
  actions.ClearSelectionRouteCooling();
  explorer_.ClearFailedBlacklist();
  completion_.ResetBudget();
  ResetBlocked();
  ResetRejected();
  entry_.Clear();
  actions.DiscardRescueNavigation();
  if (actions.TipPinchedNow() && actions.EscapeOrShortRetreat("tip rescue; peel into free",
                                                              "tip rescue; short trail retreat")) {
    actions.DeferRescueRecovery(reason);
    return true;
  }
  actions.BuildTour();
  // Build can refresh mission observations. Read the pose at the original
  // prioritization boundary, rather than retaining the initial gate snapshot.
  if (!tour_.PrioritizeDistant(explorer_, *actions.TourRobot(), actions)) {
    actions.ReportSelectionRescue(reason, true);
    return false;
  }
  mission.ResumeExploring();
  actions.SelectNextGoal();
  return actions.HasRescueNavigation() || entry_.pending() || entry_.locked();
}
}  // namespace navigation2d::exploration
