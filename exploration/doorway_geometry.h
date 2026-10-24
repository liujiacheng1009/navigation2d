#pragma once
#include <algorithm>
#include <cmath>

#include "navigation2d/exploration/frontier_types.h"
#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::frontier_internal {
inline double OccupiedClearanceAt(const ExplorationGrid& map, int col, int row) {
  if (map.resolution <= 0. || col < 0 || row < 0 || col >= map.width || row >= map.height) {
    return 0.;
  }
  if (map.cells[static_cast<std::size_t>(row) * map.width + col] != 0) {
    return 0.;
  }
  const double wx = map.origin_x + (col + .5) * map.resolution;
  const double wy = map.origin_y + (row + .5) * map.resolution;
  const double half = .5 * map.resolution;
  const int reach = static_cast<int>(std::ceil(.55 / map.resolution));
  double best = .55;
  for (int dy = -reach; dy <= reach; ++dy) {
    for (int dx = -reach; dx <= reach; ++dx) {
      const int x = col + dx, y = row + dy;
      if (x < 0 || y < 0 || x >= map.width || y >= map.height) {
        best = std::min(
            best, std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * map.resolution);
        continue;
      }
      if (map.cells[static_cast<std::size_t>(y) * map.width + x] <= 0) {
        continue;
      }
      const double ox = map.origin_x + (x + .5) * map.resolution;
      const double oy = map.origin_y + (y + .5) * map.resolution;
      const double nearest_x = std::max(std::abs(wx - ox) - half, 0.);
      const double nearest_y = std::max(std::abs(wy - oy) - half, 0.);
      best = std::min(best, std::hypot(nearest_x, nearest_y));
    }
  }
  return best;
}
inline bool UnknownRoomBeyond(const ExplorationGrid& map, int col, int row) {
  if (map.resolution <= 0.) {
    return false;
  }
  const int need = std::max(1, static_cast<int>(std::ceil(.55 / map.resolution)));
  auto value = [&](int c, int r) -> std::int8_t {
    if (c < 0 || r < 0 || c >= map.width || r >= map.height) {
      return 100;
    }
    return map.cells[static_cast<std::size_t>(r) * map.width + c];
  };
  int max_depth = 0;
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  for (int direction = 0; direction < 4; ++direction) {
    const int step_c = dx[direction];
    const int step_r = dy[direction];
    if (value(col + step_c, row + step_r) >= 0) {
      continue;
    }
    int depth = 0;
    for (int step = 1; step <= need + 2; ++step) {
      if (value(col + step * step_c, row + step * step_r) >= 0) {
        break;
      }
      depth = step;
    }
    max_depth = std::max(max_depth, depth);
  }
  return max_depth >= need;
}
inline bool CorridorMouthAt(const ExplorationGrid& map, double frontier_x, double frontier_y) {
  if (map.resolution <= 0. || map.width <= 0 || map.height <= 0) {
    return false;
  }
  const int col = static_cast<int>(std::floor((frontier_x - map.origin_x) / map.resolution));
  const int row = static_cast<int>(std::floor((frontier_y - map.origin_y) / map.resolution));
  if (col < 0 || row < 0 || col >= map.width || row >= map.height) {
    return false;
  }
  auto free = [&](int c, int r) {
    if (c < 0 || r < 0 || c >= map.width || r >= map.height) {
      return false;
    }
    return map.cells[static_cast<std::size_t>(r) * map.width + c] == 0;
  };
  const int probe = static_cast<int>(std::ceil(1.4 / map.resolution));
  int horizontal = 0, vertical = 0;
  for (int x = col; x >= 0 && x > col - probe && free(x, row); --x) {
    ++horizontal;
  }
  for (int x = col + 1; x < map.width && x <= col + probe && free(x, row); ++x) {
    ++horizontal;
  }
  for (int y = row; y >= 0 && y > row - probe && free(col, y); --y) {
    ++vertical;
  }
  for (int y = row + 1; y < map.height && y <= row + probe && free(col, y); ++y) {
    ++vertical;
  }
  const int narrow_min =
      static_cast<int>(std::ceil(navigation2d::kNarrowMinWidth() / map.resolution));
  const int narrow_max =
      static_cast<int>(std::floor(navigation2d::kNarrowMaxWidth() / map.resolution));
  // Open-room free space hits the probe cap on BOTH axes — that is a wall
  // expansion, not a door throat (live: gap=1.65 "easy door" on open wall).
  // Exception: a still-unknown room behind a wide free gap also caps both
  // axes after the throat is flooded; keep those as mouths so entry is not
  // abandoned mid-approach (live: frontier≈(-2.20,2.49) → candidates=0).
  if (horizontal >= narrow_max && vertical >= narrow_max) {
    return UnknownRoomBeyond(map, col, row);
  }
  const bool narrow_h = horizontal >= narrow_min && horizontal <= narrow_max;
  const bool narrow_v = vertical >= narrow_min && vertical <= narrow_max;
  const int smaller = std::min(horizontal, vertical);
  return narrow_h || narrow_v || (smaller >= narrow_min && smaller <= narrow_max);
}
}  // namespace navigation2d::frontier_internal
