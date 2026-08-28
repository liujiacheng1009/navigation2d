#include "navigation2d/planning/theta_star_planner.h"

#include <cmath>

#include "navigation2d/planning/grid_search.h"

namespace navigation2d {

Path ThetaStarPlanner::Plan(const LayeredCostmap& costmap, const Pose2d& start,
                            const Pose2d& goal) const {
  const Grid2d& grid = costmap.grid();
  Path path = planning_internal::GridSearch(
      costmap, start, goal, clearance_,
      [](int x, int y, int gx, int gy) { return std::hypot(gx - x, gy - y); },
      [this](const LayeredCostmap& map, int parent, int next) {
        return planning_internal::SegmentTraversalCost(map, parent, next, clearance_);
      });
  return planning_internal::DensifyPath(path, 2. * grid.resolution());
}

}  // namespace navigation2d
