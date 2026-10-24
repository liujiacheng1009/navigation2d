#include "navigation2d/application/exploration/entry_crawl_executor.h"

#include <cassert>
#include <cmath>

#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
bool HasActiveEntryCrawl(const EntryCoordinator& entry) {
  return entry.pending() && entry.crawl_ticks() > 0 && entry.crawl_ticks() <= 40;
}
void ExecuteEntryCrawl(EntryCoordinator& entry, const FrontierExplorer& explorer,
                       const ExplorationGoal& target, TourPose robot, EntryCrawlActions& actions) {
  assert(HasActiveEntryCrawl(entry));
  if (entry.Satisfied(robot.x, robot.y, target, explorer)) {
    actions.AcknowledgeEntry(EntryArrival::kCrawl);
    return;
  }
  const bool blocked = std::hypot(target.x - robot.x, target.y - robot.y) <= kPlanningRadius() ||
                       actions.CrawlSegmentCutsWall(robot.x, robot.y, target);
  const auto step = entry.AdvanceCrawl({robot.x, robot.y, robot.yaw, blocked}, target);
  if (step.state == EntryCrawlStep::State::kBlocked) {
    actions.AbandonLockedEntryMouth("open-loop crawl blocked by wall; leaving mouth");
    return;
  }
  actions.PublishEntryCrawl(step.command);
  if (step.state == EntryCrawlStep::State::kLastCommand) {
    entry.StopCrawl();
    if (entry.attempts() < entry.AttemptBudget(explorer) && actions.StartEntryProbe()) {
      return;
    }
    actions.AbandonLockedEntryMouth("open-loop crawl exhausted; leaving mouth");
  }
}
}  // namespace navigation2d::exploration
