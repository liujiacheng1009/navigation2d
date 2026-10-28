#include "navigation2d/exploration/frontier_boundary_queries.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "navigation2d/exploration/doorway_geometry.h"
#include "navigation2d/exploration/planning_grid_view.h"

namespace navigation2d::frontier_internal {
double MouthFreeDepthIn(const ExplorationGrid& planning, const ExplorationGrid& observation,
                        double footprint_clearance, const ExplorationGoal& frontier_goal,
                        double robot_x, double robot_y) {
  const auto& map = planning;
  const PlanningGridView view(planning, observation, footprint_clearance);
  if (map.resolution <= 0. || map.width <= 2 || map.height <= 2) {
    return 0.;
  }
  double dir_x = frontier_goal.frontier_x - robot_x;
  double dir_y = frontier_goal.frontier_y - robot_y;
  double len = std::hypot(dir_x, dir_y);
  if (len < .05) {
    dir_x = frontier_goal.frontier_x - frontier_goal.x;
    dir_y = frontier_goal.frontier_y - frontier_goal.y;
    len = std::hypot(dir_x, dir_y);
  }
  if (len < .05) {
    return 0.;
  }
  dir_x /= len;
  dir_y /= len;
  double depth = 0.;
  for (double step = map.resolution; step <= 1.40 + 1e-6; step += map.resolution) {
    const double x = frontier_goal.frontier_x + step * dir_x;
    const double y = frontier_goal.frontier_y + step * dir_y;
    const int col = static_cast<int>(std::floor((x - map.origin_x) / map.resolution));
    const int row = static_cast<int>(std::floor((y - map.origin_y) / map.resolution));
    if (col < 0 || row < 0 || col >= map.width || row >= map.height) {
      break;
    }
    if (!view.Free(col, row) || !view.ClearAt(col, row, .14)) {
      break;
    }
    depth = step;
  }
  return depth;
}
bool GoalStillFrontierIn(const ExplorationGrid& observation, const ExplorationGoal& goal) {
  if (observation.resolution <= 0. || observation.width <= 0 || observation.height <= 0) {
    return false;
  }
  const int col = static_cast<int>(
      std::floor((goal.frontier_x - observation.origin_x) / observation.resolution));
  const int row = static_cast<int>(
      std::floor((goal.frontier_y - observation.origin_y) / observation.resolution));
  return ObservationGridView(observation).Frontier(col, row);
}
bool UnenteredRoomBeyondIn(const ExplorationGrid& observation, const ExplorationGoal& goal) {
  const ExplorationGrid& map = observation;
  if (map.resolution <= 0. || map.width <= 2 || map.height <= 2 ||
      map.cells.size() != static_cast<std::size_t>(map.width) * map.height) {
    return false;
  }
  const double dx = goal.frontier_x - goal.x;
  const double dy = goal.frontier_y - goal.y;
  const double length = std::hypot(dx, dy);
  if (length < map.resolution) {
    return false;
  }
  const double ux = dx / length;
  const double uy = dy / length;
  // Same distances already used to tell a room from a fringe: CorridorMouthAt's
  // 1.4 m free-run probe, and UnknownRoomBeyond's 0.55 m unknown depth.
  const int steps = static_cast<int>(std::ceil(1.4 / map.resolution));
  const int need = std::max(1, static_cast<int>(std::ceil(.55 / map.resolution)));
  auto at = [&](int col, int row) -> std::int8_t {
    if (col < 0 || row < 0 || col >= map.width || row >= map.height) {
      return 100;
    }
    return map.cells[static_cast<std::size_t>(row) * map.width + col];
  };
  bool have_free = false;
  double free_x = 0.;
  double free_y = 0.;
  int unknown_run = 0;
  for (int step = -1; step <= steps; ++step) {
    const double wx = goal.frontier_x + step * map.resolution * ux;
    const double wy = goal.frontier_y + step * map.resolution * uy;
    const int col = static_cast<int>(std::floor((wx - map.origin_x) / map.resolution));
    const int row = static_cast<int>(std::floor((wy - map.origin_y) / map.resolution));
    const auto cell = at(col, row);
    if (cell > 0) {
      // A jamb beside the standoff is not the room. Occupied on the
      // into-room ray means this axis is a wall, not an entrance.
      if (step < 0) {
        continue;
      }
      return false;
    }
    if (cell == 0) {
      have_free = true;
      free_x = map.origin_x + (col + .5) * map.resolution;
      free_y = map.origin_y + (row + .5) * map.resolution;
      unknown_run = 0;
      continue;
    }
    ++unknown_run;
    if (have_free && unknown_run >= need && CorridorMouthAt(map, free_x, free_y)) {
      return true;
    }
  }
  return false;
}
bool GoalRegionStillFrontierIn(const ExplorationGrid& observation, const ExplorationGoal& goal) {
  if (GoalStillFrontierIn(observation, goal) || observation.resolution <= 0.) {
    return GoalStillFrontierIn(observation, goal);
  }
  const int center_col = static_cast<int>(
      std::floor((goal.frontier_x - observation.origin_x) / observation.resolution));
  const int center_row = static_cast<int>(
      std::floor((goal.frontier_y - observation.origin_y) / observation.resolution));
  // The association radius is deliberately smaller than the viewpoint
  // standoff. It follows a shifted sensor boundary but cannot jump across a
  // shelf to an unrelated frontier on its far side.
  const int radius = static_cast<int>(std::ceil(.35 / observation.resolution));
  for (int row = std::max(0, center_row - radius);
       row <= std::min(observation.height - 1, center_row + radius); ++row) {
    for (int col = std::max(0, center_col - radius);
         col <= std::min(observation.width - 1, center_col + radius); ++col) {
      if (std::hypot(col - center_col, row - center_row) > radius) {
        continue;
      }
      if (ObservationGridView(observation).Frontier(col, row)) {
        return true;
      }
    }
  }
  return false;
}
}  // namespace navigation2d::frontier_internal
