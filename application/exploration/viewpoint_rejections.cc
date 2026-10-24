#include "navigation2d/application/exploration/viewpoint_rejections.h"

#include <cmath>
namespace navigation2d::exploration {
bool ViewpointRejections::Nearby(const ExplorationGoal& goal, const ExplorationGoal& anchor) {
  // A doorway cluster spans more than one cell; pose or frontier proximity
  // matches the same pocket. Keep strict boundaries and the original anchor.
  return std::hypot(goal.x - anchor.x, goal.y - anchor.y) < 1.20 ||
         std::hypot(goal.frontier_x - anchor.frontier_x, goal.frontier_y - anchor.frontier_y) <
             1.50;
}
int ViewpointRejections::NoteScrape(const ExplorationGoal& goal) {
  if (!scrape_goal_ || !Nearby(goal, *scrape_goal_)) {
    scrape_goal_ = goal;
    scrape_count_ = 1;
  } else {
    ++scrape_count_;
  }
  return scrape_count_;
}
void ViewpointRejections::ResetScrapes() {
  scrape_goal_.reset();
  scrape_count_ = 0;
}
void ViewpointRejections::Demote(const ExplorationGoal& goal, std::int64_t until_ns) {
  demoted_ = Demotion{goal, until_ns};
}
bool ViewpointRejections::RecentlyDemoted(const ExplorationGoal& goal, std::int64_t now_ns) const {
  if (!HasDemotion() || now_ns >= demoted_->until_ns) {
    return false;
  }
  return Nearby(goal, demoted_->goal);
}
}  // namespace navigation2d::exploration
