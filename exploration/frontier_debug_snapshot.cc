#include "navigation2d/exploration/frontier_debug_snapshot.h"

#include <cmath>
#include <limits>

#include "navigation2d/exploration/observation_grid_view.h"

namespace navigation2d::frontier_internal {
void RefreshFrontierDebugCells(const ExplorationGrid& observation,
                               FrontierDebugSnapshot& snapshot) {
  snapshot.cells.clear();
  if (observation.resolution <= 0. || observation.width <= 2 || observation.height <= 2 ||
      observation.cells.size() !=
          static_cast<std::size_t>(observation.width * observation.height)) {
    return;
  }
  const int width = observation.width;
  const int height = observation.height;
  // Color live RawFrontier cells by nearest last-SelectGoals cluster goal so
  // the FR overlay stays tied to CG markers (independent recluster ids made
  // C# labels look unrelated to CG/SG).
  constexpr double kAssociateM = 1.5;
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      if (!ObservationGridView(observation).Frontier(col, row)) {
        continue;
      }
      const double x = observation.origin_x + (col + .5) * observation.resolution;
      const double y = observation.origin_y + (row + .5) * observation.resolution;
      int best_id = 0;
      double best_d = kAssociateM;
      for (const auto& cg : snapshot.cluster_goals) {
        const double d = std::hypot(cg.frontier_x - x, cg.frontier_y - y);
        if (d < best_d) {
          best_d = d;
          best_id = cg.cluster_id;
        }
      }
      snapshot.cells.push_back({x, y, best_id});
    }
  }
}
void AppendStableFrontierDebug(FrontierDebugSnapshot& snapshot,
                               std::span<const ExplorationGoal> stable) {
  const auto nearest_cluster_id = [&](double fx, double fy) {
    int best_id = -1;
    double best_d = std::numeric_limits<double>::infinity();
    for (const auto& cg : snapshot.cluster_goals) {
      const double d = std::hypot(cg.frontier_x - fx, cg.frontier_y - fy);
      if (d < best_d) {
        best_d = d;
        best_id = cg.cluster_id;
      }
    }
    return best_id;
  };
  for (const auto& goal : stable) {
    snapshot.goals.push_back({goal.x, goal.y, goal.frontier_x, goal.frontier_y,
                              nearest_cluster_id(goal.frontier_x, goal.frontier_y), goal.score,
                              goal.information_gain, goal.frontier_cells});
  }
}
}  // namespace navigation2d::frontier_internal
