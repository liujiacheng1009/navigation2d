#pragma once
#include <string>

#include "navigation2d/application/exploration/entry_actions.h"
#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/tour_order.h"
namespace navigation2d::exploration {
class EntryCrawlActions {
 public:
  virtual ~EntryCrawlActions() = default;
  virtual void AcknowledgeEntry(EntryArrival) = 0;
  virtual bool StartEntryProbe() = 0;
  virtual void AbandonLockedEntryMouth(const std::string& reason) = 0;
  virtual bool CrawlSegmentCutsWall(double x, double y, const ExplorationGoal&) const = 0;
  // Publish the command, then the legacy completing status, synchronously.
  virtual void PublishEntryCrawl(Twist2d) = 0;
};
[[nodiscard]] bool HasActiveEntryCrawl(const EntryCoordinator&);
// Active crawl and valid world-frame pose required. Check arrival before
// querying the live segment, and publish the final command before retrying or
// abandoning. EntryCoordinator owns tick/attempt/lock state; no references stay.
void ExecuteEntryCrawl(EntryCoordinator&, const FrontierExplorer&, const ExplorationGoal& target,
                       TourPose robot, EntryCrawlActions&);
}  // namespace navigation2d::exploration
