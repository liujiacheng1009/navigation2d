#include <algorithm>
#include <cmath>

#include "navigation2d/exploration/entry_probe_search.h"
namespace navigation2d::frontier_internal {
double MouthGapWidthAt(const ExplorationGrid& map, double frontier_x, double frontier_y) {
  if (map.resolution <= 0. || map.width <= 0 || map.height <= 0) {
    return 0.;
  }
  const int col = static_cast<int>(std::floor((frontier_x - map.origin_x) / map.resolution));
  const int row = static_cast<int>(std::floor((frontier_y - map.origin_y) / map.resolution));
  if (col < 0 || row < 0 || col >= map.width || row >= map.height) {
    return 0.;
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
  // Throat width is the narrow free run (door gap), not the deep room axis.
  return static_cast<double>(std::min(horizontal, vertical)) * map.resolution;
}
}  // namespace navigation2d::frontier_internal
