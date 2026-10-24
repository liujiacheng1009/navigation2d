#include <algorithm>
#include <cmath>

#include "navigation2d/exploration/entry_probe_search.h"
namespace navigation2d::frontier_internal {
std::optional<std::pair<double, double>> EntryProbeDirection(const ExplorationGrid& map,
                                                             const ExplorationGrid& observation,
                                                             const ExplorationGoal& frontier_goal,
                                                             double robot_x, double robot_y) {
  // Entry axis must point into the unexplored room. Viewpoint→frontier is
  // often lateral (standoff beside the jamb), which placed the probe along
  // the hall wall and left the robot stuck on the threshold with CARROT
  // inside the room. Prefer the raw unknown centroid (observation map).
  double direction_x = 0.;
  double direction_y = 0.;
  {
    const ExplorationGrid& obs = (observation.width == map.width &&
                                  observation.height == map.height && observation.resolution > 0.)
                                     ? observation
                                     : map;
    const int fcol =
        static_cast<int>(std::floor((frontier_goal.frontier_x - obs.origin_x) / obs.resolution));
    const int frow =
        static_cast<int>(std::floor((frontier_goal.frontier_y - obs.origin_y) / obs.resolution));
    const int reach = static_cast<int>(std::ceil(1.20 / obs.resolution));
    double unknown_x = 0.;
    double unknown_y = 0.;
    int unknown_n = 0;
    for (int row = std::max(0, frow - reach); row <= std::min(obs.height - 1, frow + reach);
         ++row) {
      for (int col = std::max(0, fcol - reach); col <= std::min(obs.width - 1, fcol + reach);
           ++col) {
        if (obs.cells[static_cast<std::size_t>(row) * obs.width + col] >= 0) {
          continue;
        }
        const double x = obs.origin_x + (col + .5) * obs.resolution;
        const double y = obs.origin_y + (row + .5) * obs.resolution;
        unknown_x += x - frontier_goal.frontier_x;
        unknown_y += y - frontier_goal.frontier_y;
        ++unknown_n;
      }
    }
    if (unknown_n >= 3) {
      direction_x = unknown_x / static_cast<double>(unknown_n);
      direction_y = unknown_y / static_cast<double>(unknown_n);
    }
  }
  if (std::hypot(direction_x, direction_y) < .05) {
    direction_x = frontier_goal.frontier_x - robot_x;
    direction_y = frontier_goal.frontier_y - robot_y;
  }
  if (std::hypot(direction_x, direction_y) < .05) {
    direction_x = frontier_goal.frontier_x - frontier_goal.x;
    direction_y = frontier_goal.frontier_y - frontier_goal.y;
  }
  const double approach_length = std::hypot(direction_x, direction_y);
  if (approach_length < .05) {
    return std::nullopt;
  }
  direction_x /= approach_length;
  direction_y /= approach_length;
  return std::pair{direction_x, direction_y};
}

}  // namespace navigation2d::frontier_internal
