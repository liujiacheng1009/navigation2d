#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

#include "navigation2d/exploration/entry_probe_search.h"
namespace navigation2d::frontier_internal {
bool EntryProbeGrid::Allows(int col, int row) const {
  const ObservationGridView observed(observation_);
  const auto near_observed_free = [&](int col, int row) {
    if (observed.Free(col, row)) {
      return true;
    }
    // Allow cells that touch observed free so probes stay near mapped space.
    for (int dy = -2; dy <= 2; ++dy) {
      for (int dx = -2; dx <= 2; ++dx) {
        if (observed.Free(col + dx, row + dy)) {
          return true;
        }
      }
    }
    return false;
  };
  return geometry_.Free(col, row) && geometry_.ClearAt(col, row, kPlanningRadius()) &&
         near_observed_free(col, row);
}

std::vector<int> EntryProbeGrid::Distances(double robot_x, double robot_y) const {
  const auto& map = map_;
  const int robot_col = static_cast<int>(std::floor((robot_x - map.origin_x) / map.resolution));
  const int robot_row = static_cast<int>(std::floor((robot_y - map.origin_y) / map.resolution));
  const int start_search = static_cast<int>(std::ceil(1.0 / map.resolution));
  int start = -1;
  double start_distance = std::numeric_limits<double>::infinity();
  for (int row = std::max(0, robot_row - start_search);
       row <= std::min(map.height - 1, robot_row + start_search); ++row) {
    for (int col = std::max(0, robot_col - start_search);
         col <= std::min(map.width - 1, robot_col + start_search); ++col) {
      if (!Allows(col, row)) {
        continue;
      }
      const double distance = std::hypot(col - robot_col, row - robot_row);
      if (distance < start_distance) {
        start_distance = distance;
        start = row * map.width + col;
      }
    }
  }
  if (start < 0) {
    return {};
  }

  // Entry probe walks the planning map (same occupancy as the mapper).
  std::vector<int> distance(map.cells.size(), -1);
  std::deque<int> queue{start};
  distance[start] = 0;
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  while (!queue.empty()) {
    const int current = queue.front();
    queue.pop_front();
    const int row = current / map.width;
    const int col = current % map.width;
    for (int direction = 0; direction < 4; ++direction) {
      const int next_col = col + dx[direction];
      const int next_row = row + dy[direction];
      if (next_col < 0 || next_row < 0 || next_col >= map.width || next_row >= map.height) {
        continue;
      }
      const int next = next_row * map.width + next_col;
      if (distance[next] >= 0 || !Allows(next_col, next_row)) {
        continue;
      }
      distance[next] = distance[current] + 1;
      queue.push_back(next);
    }
  }

  return distance;
}

}  // namespace navigation2d::frontier_internal
