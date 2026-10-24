#pragma once
#include <optional>
#include <string>

#include "navigation2d/application/exploration/entry_actions.h"
#include "navigation2d/application/exploration/tour_order.h"
#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d {
class FrontierExplorer;
namespace exploration {
class EntryCoordinator;
enum class EntrySelectionResult { kContinueTour, kHandled };
enum class EntrySelectionDelay { kWideMouthCrawl, kRejectedPlan };
class EntrySelectionActions {
 public:
  virtual ~EntrySelectionActions() = default;
  virtual bool StartEntryProbe() = 0;
  virtual void AcknowledgeEntry(EntryArrival) = 0;
  virtual void AbandonLockedEntryMouth(const std::string& reason) = 0;
  virtual void DeferEntrySelection(EntrySelectionDelay) = 0;
};
// Synchronous selection gate. Entry owns lock/budget/crawl; actions perform
// navigation/arrival and ROS output without retaining references. A failed
// probe can update entry state, so the remaining budget is read after it.
// No pose means arrival cannot be acknowledged. Pose uses world metres.
[[nodiscard]] EntrySelectionResult SelectLockedEntry(EntryCoordinator&, const FrontierExplorer&,
                                                     const std::optional<TourPose>& robot,
                                                     const ExplorationGoal& active_goal,
                                                     EntrySelectionActions&);
}  // namespace exploration
}  // namespace navigation2d
