#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>

#include "navigation2d/exploration/unvisited_space.h"
namespace navigation2d::frontier_internal {
// `ascend` limits the flood to steps that do not get closer to the trail,
// so a seed spreads only into the space behind its door.
std::vector<double> FreeSpaceDistances(const ExplorationGrid& map,
                                       const std::vector<std::pair<double, double>>& sources,
                                       double limit_m, int snap_cells,
                                       const std::vector<double>* ascend) {
  const int n = map.width * map.height;
  const double inf = std::numeric_limits<double>::infinity();
  constexpr int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
  constexpr int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
  const double step[] = {1.,           1., 1., 1., std::sqrt(2.), std::sqrt(2.), std::sqrt(2.),
                         std::sqrt(2.)};
  const auto free = [&](int col, int row) {
    return col >= 0 && row >= 0 && col < map.width && row < map.height &&
           map.cells[static_cast<std::size_t>(row) * map.width + col] == 0;
  };
  std::vector<double> distance(n, inf);
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  for (const auto& [x, y] : sources) {
    const int col = static_cast<int>(std::floor((x - map.origin_x) / map.resolution));
    const int row = static_cast<int>(std::floor((y - map.origin_y) / map.resolution));
    int best = -1;
    double best_d = inf;
    for (int r = row - snap_cells; r <= row + snap_cells; ++r) {
      for (int c = col - snap_cells; c <= col + snap_cells; ++c) {
        if (!free(c, r)) {
          continue;
        }
        const double d = std::hypot(c - col, r - row) * map.resolution;
        if (d < best_d) {
          best_d = d;
          best = r * map.width + c;
        }
      }
    }
    if (best >= 0 && distance[best] > 0.) {
      distance[best] = 0.;
      open.emplace(0., best);
    }
  }
  while (!open.empty()) {
    const auto [d, i] = open.top();
    open.pop();
    if (d > distance[i]) {
      continue;
    }
    const int col = i % map.width, row = i / map.width;
    for (int k = 0; k < 8; ++k) {
      const int c = col + dx[k], r = row + dy[k];
      if (!free(c, r)) {
        continue;
      }
      const int j = r * map.width + c;
      if (ascend && (*ascend)[j] < (*ascend)[i]) {
        continue;
      }
      const double next = d + step[k] * map.resolution;
      if (next <= limit_m && next < distance[j]) {
        distance[j] = next;
        open.emplace(next, j);
      }
    }
  }
  return distance;
}

}  // namespace navigation2d::frontier_internal
