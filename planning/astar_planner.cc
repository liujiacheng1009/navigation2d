#include "navigation2/planning/astar_planner.h"

#include <cmath>

#include "navigation2/planning/grid_search.h"

namespace navigation2d {

Path AStarPlanner::Plan(const LayeredCostmap& costmap, const Pose2d& start,
                        const Pose2d& goal) const {
  return planning_internal::GridSearch(
      costmap, start, goal, clearance_,
      [](int x, int y, int gx, int gy) { return std::hypot(gx - x, gy - y); });
}

}  // namespace navigation2d
