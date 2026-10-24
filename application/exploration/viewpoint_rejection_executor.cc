#include "navigation2d/application/exploration/viewpoint_rejection_executor.h"

#include "navigation2d/application/exploration/tour_execution.h"
#include "navigation2d/application/exploration/viewpoint_rejections.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
void RejectOpenWallViewpoint(FrontierExplorer& explorer, TourExecution& tour,
                             ViewpointRejections& history, const RejectedViewpoint& rejected,
                             ViewpointRejectionActions& actions) {
  const auto& goal = rejected.goal;
  const bool scrape = rejected.reason.find("wall-scrape") != std::string::npos ||
                      rejected.reason.find("tip-scrape") != std::string::npos;
  if (scrape) {
    if (rejected.path_signature != 0) {
      actions.CoolRejectedSignature(rejected.path_signature);
    }
    const int count = history.NoteScrape(goal);
    if (count < 2) {
      // A transient scrape rotates the tour without inflating failed-goal counts.
      tour.RetryLater(goal);
      actions.ReportViewpointRejection(
          {ViewpointRejectionKind::kSoftScrape, goal, rejected.reason, count});
      return;
    }
  }
  explorer.RecordAttempt(goal, false, true, false);
  history.Demote(goal, actions.RejectionDeadlineNs(10.0));
  history.ResetScrapes();
  actions.ReportViewpointRejection({ViewpointRejectionKind::kDemoted, goal, rejected.reason});
}
}  // namespace navigation2d::exploration
