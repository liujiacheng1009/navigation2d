#include "navigation2/planning/dijkstra_planner.h"

#include "navigation2/planning/grid_search.h"

namespace navigation2d {

Path DijkstraPlanner::Plan(const LayeredCostmap& costmap, const Pose2d& start,
                           const Pose2d& goal) const {
  return planning_internal::GridSearch(
      costmap, start, goal, clearance_,
      [](int, int, int, int) { return 0.; });
}

}  // namespace navigation2d
