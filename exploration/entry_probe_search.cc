#include "navigation2d/exploration/entry_probe_search.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "navigation2d/exploration/doorway_geometry.h"
namespace navigation2d::frontier_internal {
std::optional<ExplorationGoal> SelectEntryProbeAt(const ExplorationGrid& map,
                                                  const ExplorationGrid& observation,
                                                  double footprint_clearance,
                                                  const ExplorationGoal& frontier_goal,
                                                  double robot_x, double robot_y,
                                                  double minimum_depth_m, double maximum_depth_m) {
  if (map.width <= 2 || map.height <= 2 || map.resolution <= 0. ||
      map.cells.size() != static_cast<std::size_t>(map.width * map.height) ||
      minimum_depth_m <= 0. || maximum_depth_m < minimum_depth_m) {
    return std::nullopt;
  }

  const auto direction = EntryProbeDirection(map, observation, frontier_goal, robot_x, robot_y);
  if (!direction) {
    return std::nullopt;
  }
  const auto [direction_x, direction_y] = *direction;
  const EntryProbeGrid geometry(map, observation, footprint_clearance);
  const auto distance = geometry.Distances(robot_x, robot_y);
  if (distance.empty()) {
    return std::nullopt;
  }
  std::optional<ExplorationGoal> best;
  double best_score = -std::numeric_limits<double>::infinity();
  const int search = static_cast<int>(std::ceil(maximum_depth_m / map.resolution)) + 2;
  const int frontier_col =
      static_cast<int>(std::floor((frontier_goal.frontier_x - map.origin_x) / map.resolution));
  const int frontier_row =
      static_cast<int>(std::floor((frontier_goal.frontier_y - map.origin_y) / map.resolution));
  for (int row = std::max(0, frontier_row - search);
       row <= std::min(map.height - 1, frontier_row + search); ++row) {
    for (int col = std::max(0, frontier_col - search);
         col <= std::min(map.width - 1, frontier_col + search); ++col) {
      const int index = row * map.width + col;
      if (distance[index] < 0 || !geometry.Allows(col, row)) {
        continue;
      }
      const auto [x, y] = std::pair{map.origin_x + (col + .5) * map.resolution,
                                    map.origin_y + (row + .5) * map.resolution};
      const double relative_x = x - frontier_goal.frontier_x;
      const double relative_y = y - frontier_goal.frontier_y;
      const double depth = relative_x * direction_x + relative_y * direction_y;
      const double lateral = std::abs(relative_x * direction_y - relative_y * direction_x);
      // Keep probes in the door throat center — .38 m lateral parked on the
      // jamb (UI: GOAL on left threshold wall, path clear ~0.06 m). Wide easy
      // mouths allow a hair more lateral so angled hall approaches still land
      // mid-gap instead of returning nullopt and abandoning the door.
      const double gap = MouthGapWidthAt(map, frontier_goal.frontier_x, frontier_goal.frontier_y);
      const double max_lateral = gap >= .50 ? .28 : .20;
      if (depth < minimum_depth_m || depth > maximum_depth_m || lateral > max_lateral) {
        continue;
      }
      // Do not select a pose that is merely on the approach side of the
      // frontier or an unrelated free cell behind a nearby wall.
      if (std::hypot(relative_x, relative_y) > maximum_depth_m + .20) {
        continue;
      }
      const double clearance = OccupiedClearanceAt(map, col, row);
      if (clearance < kPlanningRadius()) {
        continue;
      }
      // Clearance dominates: mid-gap beats a deeper jamb-hugging cell.
      const double score = 0.35 * depth + 5.5 * clearance - 3.0 * lateral -
                           .01 * static_cast<double>(distance[index]);
      if (score > best_score) {
        best_score = score;
        ExplorationGoal probe = frontier_goal;
        probe.x = x;
        probe.y = y;
        probe.yaw = std::atan2(direction_y, direction_x);
        probe.score = score;
        best = probe;
      }
    }
  }
  if (best) {
    return best;
  }
  // Geometric fallback: place the probe on the approach axis when BFS cannot
  // reach a candidate. Prefer the lateral with the largest jamb clearance.
  const double lateral_unit_x = -direction_y;
  const double lateral_unit_y = direction_x;
  std::optional<ExplorationGoal> fallback;
  double fallback_clear = -1.;
  for (double depth = minimum_depth_m; depth <= maximum_depth_m + 1e-6;
       depth += std::max(.05, map.resolution)) {
    for (const double lateral : {0., .08, -.08, .14, -.14}) {
      const double x = frontier_goal.frontier_x + depth * direction_x + lateral * lateral_unit_x;
      const double y = frontier_goal.frontier_y + depth * direction_y + lateral * lateral_unit_y;
      const int col = static_cast<int>(std::floor((x - map.origin_x) / map.resolution));
      const int row = static_cast<int>(std::floor((y - map.origin_y) / map.resolution));
      if (!geometry.Allows(col, row)) {
        continue;
      }
      const double clearance = OccupiedClearanceAt(map, col, row);
      if (clearance < kPlanningRadius()) {
        continue;
      }
      if (clearance > fallback_clear) {
        fallback_clear = clearance;
        ExplorationGoal probe = frontier_goal;
        probe.x = map.origin_x + (col + .5) * map.resolution;
        probe.y = map.origin_y + (row + .5) * map.resolution;
        probe.yaw = std::atan2(direction_y, direction_x);
        probe.score = depth + 4.5 * clearance;
        fallback = probe;
      }
    }
    if (fallback && fallback_clear >= kPlanningRadius()) {
      return fallback;
    }
  }
  return fallback;
}
}  // namespace navigation2d::frontier_internal
