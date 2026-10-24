#pragma once
#include <cstdint>
#include <string>

#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d {
class FrontierExplorer;
namespace exploration {
class TourExecution;
class ViewpointRejections;
enum class ViewpointRejectionKind { kSoftScrape, kDemoted };
struct RejectedViewpoint {
  ExplorationGoal goal;
  const std::string& reason;  // Synchronous borrow, never retained.
  std::uint64_t path_signature;
};
struct ViewpointRejectionNotice {
  ViewpointRejectionKind kind;
  ExplorationGoal goal;
  const std::string& reason;
  int scrape_count = 0;
};
class ViewpointRejectionActions {
 public:
  virtual ~ViewpointRejectionActions() = default;
  virtual void CoolRejectedSignature(std::uint64_t) = 0;
  virtual std::int64_t RejectionDeadlineNs(double seconds) const = 0;
  // Adapters retain legacy lazy log arguments, including diagnostic geometry.
  virtual void ReportViewpointRejection(const ViewpointRejectionNotice&) = 0;
};
// Caller already consumed the rejected current tour goal. This synchronous
// flow may append a soft retry, or record a hard failure and demote the pose.
// No entry lock is acquired and no action/reference is retained.
void RejectOpenWallViewpoint(FrontierExplorer&, TourExecution&, ViewpointRejections&,
                             const RejectedViewpoint&, ViewpointRejectionActions&);
}  // namespace exploration
}  // namespace navigation2d
