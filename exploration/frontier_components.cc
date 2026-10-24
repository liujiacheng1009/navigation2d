#include "navigation2d/exploration/frontier_components.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <deque>

#include "navigation2d/exploration/observation_grid_view.h"
namespace navigation2d::frontier_internal {
FrontierComponents::FrontierComponents(int width, int height)
    : width_(width),
      height_(height),
      frontier_(static_cast<std::size_t>(width * height), 0),
      visited_(frontier_.size(), 0) {}
std::size_t FrontierComponents::Discover(const ExplorationGrid& observation,
                                         std::span<const int> travel) {
  assert(travel.size() == frontier_.size());
  if (discovered_) {
    std::fill(frontier_.begin(), frontier_.end(), 0);
    std::fill(visited_.begin(), visited_.end(), 0);
  }
  discovered_ = true;
  next_seed_ = 0;
  const ObservationGridView raw(observation);
  const auto approachable = [&](int col, int row) {
    if (!raw.Frontier(col, row)) {
      return false;
    }
    const int index = row * width_ + col;
    if (travel[static_cast<std::size_t>(index)] >= 0) {
      return true;
    }
    constexpr int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    constexpr int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
    for (int direction = 0; direction < 8; ++direction) {
      const int nc = col + dx[direction], nr = row + dy[direction];
      if (nc < 0 || nr < 0 || nc >= width_ || nr >= height_) {
        continue;
      }
      if (travel[static_cast<std::size_t>(nr * width_ + nc)] >= 0) {
        return true;
      }
    }
    return false;
  };
  for (int row = 0; row < height_; ++row) {
    for (int col = 0; col < width_; ++col) {
      if (approachable(col, row)) {
        frontier_[row * width_ + col] = 1;
      }
    }
  }
  // Preserve the separate second pass and raw evidence authorization.
  std::size_t count = 0;
  for (int row = 0; row < height_; ++row) {
    for (int col = 0; col < width_; ++col) {
      if (approachable(col, row)) {
        ++count;
      }
    }
  }
  return count;
}
std::optional<std::vector<int>> FrontierComponents::Next() {
  for (; next_seed_ < width_ * height_; ++next_seed_) {
    const int seed = next_seed_;
    if (!frontier_[seed] || visited_[seed]) {
      continue;
    }
    ++next_seed_;
    std::deque<int> queue{seed};
    std::vector<int> cluster;
    visited_[seed] = 1;
    while (!queue.empty()) {
      const int current = queue.front();
      queue.pop_front();
      cluster.push_back(current);
      const int row = current / width_, col = current % width_;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int nr = row + dy, nc = col + dx;
          if (nr < 0 || nc < 0 || nr >= height_ || nc >= width_) {
            continue;
          }
          const int next = nr * width_ + nc;
          if (frontier_[next] && !visited_[next]) {
            visited_[next] = 1;
            queue.push_back(next);
          }
        }
      }
    }
    return cluster;
  }
  return std::nullopt;
}
ClusterAnchor LocateCluster(std::span<const int> cluster, const ExplorationGrid& map) {
  assert(!cluster.empty());
  double mean_col = 0., mean_row = 0.;
  for (int index : cluster) {
    mean_col += index % map.width;
    mean_row += index / map.width;
  }
  mean_col /= cluster.size();
  mean_row /= cluster.size();
  const int representative =
      *std::min_element(cluster.begin(), cluster.end(), [&](int lhs, int rhs) {
        const double ld = std::hypot(lhs % map.width - mean_col, lhs / map.width - mean_row);
        const double rd = std::hypot(rhs % map.width - mean_col, rhs / map.width - mean_row);
        return ld < rd;
      });
  return {mean_col, mean_row, representative,
          map.origin_x + (representative % map.width + .5) * map.resolution,
          map.origin_y + (representative / map.width + .5) * map.resolution};
}
}  // namespace navigation2d::frontier_internal
