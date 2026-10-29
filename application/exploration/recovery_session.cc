#include "navigation2d/application/exploration/recovery_session.h"

#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/escape_retreat_actions.h"
#include "navigation2d/application/exploration/wall_scrape_actions.h"
#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
bool GoalRecoverySession::TryEscapeOrRetreat(const EntryCoordinator& entry,
                                             const char* escape_message,
                                             const char* retreat_message,
                                             EscapeRetreatActions& actions) {
  if (actions.TryStartEscape()) {
    resume_after_escape = true;
    actions.PublishStatus("RECOVERING", escape_message);
    return true;
  }
  // Trail reversal after a failed doorway crawl would undo entry progress.
  if (entry.pending() || entry.locked()) {
    return false;
  }
  if (actions.StartRetreat()) {
    actions.PublishStatus("NAVIGATING", retreat_message);
    return true;
  }
  return false;
}
bool GoalRecoverySession::LeavePinchAfterPeel(WallScrapeActions& actions) {
  if (wall_scrapes <= 0) {
    return false;
  }
  return actions.UnstickFromPinch("after wall peel; leave pinch pocket");
}
bool GoalRecoverySession::BreakWallScrape(const char* reason, WallScrapeActions& actions) {
  const auto route = actions.ReadScrapeRoute();
  if (route.signature != 0) {
    actions.CoolRouteSignature(route.signature);
  }
  ++wall_scrapes;
  actions.CancelRecoveryNavigation();
  actions.ReportWallScrape(reason, wall_scrapes);
  // One latest-map attempt before demotion; a previous peel or lethal tip
  // must not re-enable the same wall chord.
  if (wall_scrapes >= 2 || side_peels > 0 || !PathClearsPinch(route.clearance_m)) {
    return actions.UnstickFromPinch(reason);
  }
  const int scrapes = wall_scrapes;
  if (actions.RetryScrapedGoal()) {
    wall_scrapes = scrapes;  // A nested accepted dispatch must retain the budget.
    const auto replanned = actions.ReadScrapeRoute();
    if (!PathClearsPinch(replanned.clearance_m)) {
      if (replanned.signature != 0) {
        actions.CoolRouteSignature(replanned.signature);
      }
      ++wall_scrapes;
      return actions.UnstickFromPinch("replanned path still pinch-scrape");
    }
    actions.PublishStatus("NAVIGATING", "replanned off wall after scrape stop");
    return true;
  }
  return actions.UnstickFromPinch("pinch replan failed");
}
}  // namespace navigation2d::exploration
