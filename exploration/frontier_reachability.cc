#include "navigation2d/exploration/frontier_reachability.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>
namespace navigation2d::frontier_internal {
bool FrontierReachability::SameRegion(double anchor_x, double anchor_y, double target_x,
                                      double target_y, double max_distance_m) const {
  const auto& map = map_;
  if (map.width <= 0 || map.height <= 0 || map.resolution <= 0. ||
      map.cells.size() != static_cast<std::size_t>(map.width * map.height) || max_distance_m < 0.) {
    return false;
  }
  const auto nearest_safe = [&](double x, double y) {
    const int center_col = static_cast<int>(std::floor((x - map.origin_x) / map.resolution));
    const int center_row = static_cast<int>(std::floor((y - map.origin_y) / map.resolution));
    const int search = static_cast<int>(std::ceil(1.0 / map.resolution));
    int best = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int row = std::max(0, center_row - search);
         row <= std::min(map.height - 1, center_row + search); ++row) {
      for (int col = std::max(0, center_col - search);
           col <= std::min(map.width - 1, center_col + search); ++col) {
        if (!geometry_.SafeViewpoint(col, row)) {
          continue;
        }
        const double distance = std::hypot(col - center_col, row - center_row);
        if (distance < best_distance) {
          best_distance = distance;
          best = row * map.width + col;
        }
      }
    }
    return best;
  };
  const int start = nearest_safe(anchor_x, anchor_y);
  const int target = nearest_safe(target_x, target_y);
  if (start < 0 || target < 0) {
    return false;
  }
  const int max_cells = static_cast<int>(std::ceil(max_distance_m / map.resolution));
  std::vector<int> distance(map.cells.size(), -1);
  std::deque<int> queue{start};
  distance[start] = 0;
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  while (!queue.empty()) {
    const int current = queue.front();
    queue.pop_front();
    if (current == target) {
      return distance[current] <= max_cells;
    }
    if (distance[current] >= max_cells) {
      continue;
    }
    const int row = current / map.width, col = current % map.width;
    for (int direction = 0; direction < 4; ++direction) {
      const int next_col = col + dx[direction], next_row = row + dy[direction];
      if (next_col < 0 || next_row < 0 || next_col >= map.width || next_row >= map.height) {
        continue;
      }
      const int next = next_row * map.width + next_col;
      if (distance[next] >= 0 || !geometry_.SafeViewpoint(next_col, next_row)) {
        continue;
      }
      distance[next] = distance[current] + 1;
      queue.push_back(next);
    }
  }
  return false;
}
std::size_t FrontierReachability::UnknownCells(double robot_x, double robot_y) const {
  const auto& map = map_;
  const auto& observation = observation_;
  if (map.width <= 2 || map.height <= 2 || map.resolution <= 0. ||
      map.cells.size() != static_cast<std::size_t>(map.width * map.height) ||
      observation.cells.size() != map.cells.size()) {
    return 0;
  }
  const int robot_col = static_cast<int>(std::floor((robot_x - map.origin_x) / map.resolution));
  const int robot_row = static_cast<int>(std::floor((robot_y - map.origin_y) / map.resolution));
  // Traverse carved door noses with the narrow clearance, not full SafeViewpoint:
  // a freshly opened 0.32–0.50 m throat often fails the soft footprint floor.
  auto traversable = [&](int col, int row) {
    return geometry_.Free(col, row) && geometry_.ClearAt(col, row, kNarrowClearance());
  };
  const int search = static_cast<int>(std::ceil(1.0 / map.resolution));
  int start = -1;
  double nearest = std::numeric_limits<double>::infinity();
  for (int row = std::max(0, robot_row - search);
       row <= std::min(map.height - 1, robot_row + search); ++row) {
    for (int col = std::max(0, robot_col - search);
         col <= std::min(map.width - 1, robot_col + search); ++col) {
      if (!traversable(col, row)) {
        continue;
      }
      const double distance = std::hypot(col - robot_col, row - robot_row);
      if (distance < nearest) {
        nearest = distance;
        start = row * map.width + col;
      }
    }
  }
  if (start < 0) {
    return 0;
  }
  std::vector<unsigned char> visited(map.cells.size(), 0);
  std::deque<int> queue{start};
  visited[start] = 1;
  std::size_t reachable_unknown = 0;
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  while (!queue.empty()) {
    const int current = queue.front();
    queue.pop_front();
    if (observation.cells[static_cast<std::size_t>(current)] < 0 &&
        map.cells[static_cast<std::size_t>(current)] == 0) {
      ++reachable_unknown;
    }
    const int row = current / map.width;
    const int col = current % map.width;
    for (int direction = 0; direction < 4; ++direction) {
      const int next_col = col + dx[direction];
      const int next_row = row + dy[direction];
      if (next_col < 0 || next_row < 0 || next_col >= map.width || next_row >= map.height) {
        continue;
      }
      const int next = next_row * map.width + next_col;
      if (visited[static_cast<std::size_t>(next)] || !traversable(next_col, next_row)) {
        continue;
      }
      visited[static_cast<std::size_t>(next)] = 1;
      queue.push_back(next);
    }
  }
  return reachable_unknown;
}
}  // namespace navigation2d::frontier_internal
