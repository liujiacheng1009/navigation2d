#include "navigation2d/exploration/viewpoint_settlement.h"

#include <cmath>
#include <limits>
#include <tuple>

#include "navigation2d/exploration/planning_grid_view.h"
namespace navigation2d::frontier_internal {
std::optional<ExplorationGoal> SettleViewpoint(const ExplorationGrid& planning,
                                               const ExplorationGrid& observation,
                                               double footprint_clearance,
                                               const ExplorationGoal& goal) {
  const auto& map = planning;
  const PlanningGridView geometry(planning, observation, footprint_clearance);
  const ObservationGridView raw(observation);
  if (map.width <= 0 || map.height <= 0 || map.resolution <= 0. || !std::isfinite(goal.x) ||
      !std::isfinite(goal.y)) {
    return std::nullopt;
  }
  const double clearance = kPlanningRadius() + map.resolution;
  const int col0 = static_cast<int>(std::floor((goal.x - map.origin_x) / map.resolution));
  const int row0 = static_cast<int>(std::floor((goal.y - map.origin_y) / map.resolution));
  auto settled = [&](int col, int row) {
    return geometry.Free(col, row) && raw.Free(col, row) && geometry.ClearAt(col, row, clearance);
  };
  if (settled(col0, row0)) {
    return goal;
  }
  const int reach = static_cast<int>(std::ceil(kSwingClearance() / map.resolution));
  double best = std::numeric_limits<double>::infinity();
  int best_col = -1, best_row = -1;
  for (int row = row0 - reach; row <= row0 + reach; ++row) {
    for (int col = col0 - reach; col <= col0 + reach; ++col) {
      const double d = std::hypot(col - col0, row - row0);
      if (d > reach || d >= best || !settled(col, row)) {
        continue;
      }
      best = d;
      best_col = col;
      best_row = row;
    }
  }
  if (best_col < 0) {
    return std::nullopt;
  }
  ExplorationGoal moved = goal;
  std::tie(moved.x, moved.y) = geometry.CellCenter(best_col, best_row);
  return moved;
}
}  // namespace navigation2d::frontier_internal
