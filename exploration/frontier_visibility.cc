#include "navigation2d/exploration/frontier_visibility.h"

#include <algorithm>
#include <cmath>
namespace navigation2d::frontier_internal {
double VisibleUnknownGain(const ExplorationGrid& observation, double planning_resolution, int col,
                          int row) {
  std::vector<unsigned char> seen(observation.cells.size(), 0);
  const int max_range = static_cast<int>(std::ceil(3.0 / planning_resolution));
  for (int ray = 0; ray < 144; ++ray) {
    const double angle = 2.0 * std::acos(-1.0) * static_cast<double>(ray) / 144.0;
    for (int step = 1; step <= max_range; ++step) {
      const int x = col + static_cast<int>(std::lround(step * std::cos(angle)));
      const int y = row + static_cast<int>(std::lround(step * std::sin(angle)));
      if (x < 0 || y < 0 || x >= observation.width || y >= observation.height) {
        break;
      }
      const int index = y * observation.width + x;
      if (observation.cells[index] > 50) {
        break;
      }
      if (observation.cells[index] < 0) {
        seen[index] = 1;
      }
    }
  }
  return static_cast<double>(std::count(seen.begin(), seen.end(), 1)) * observation.resolution *
         observation.resolution;
}
}  // namespace navigation2d::frontier_internal
