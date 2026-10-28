#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/test/frontier_test_scenes.h"

int main() {
  // Diagonal corner cut: centerline misses the occupied cell but the body
  // clips the wall corner. Must still count as 穿墙 (entry body ≥0.14).
  {
    navigation2d::ExplorationGrid corner;
    corner.width = 40;
    corner.height = 40;
    corner.resolution = .05;
    corner.origin_x = 0.;
    corner.origin_y = 0.;
    corner.cells.assign(static_cast<std::size_t>(corner.width * corner.height), 0);
    // Solid L / block in the middle; path from SW free to NE free clips corner.
    for (int row = 16; row < 28; ++row) {
      for (int col = 16; col < 28; ++col) {
        corner.cells[static_cast<std::size_t>(row * corner.width + col)] = 100;
      }
    }
    const double x0 = (12 + .5) * corner.resolution;
    const double y0 = (12 + .5) * corner.resolution;
    const double x1 = (30 + .5) * corner.resolution;
    const double y1 = (30 + .5) * corner.resolution;
    const std::vector<std::pair<double, double>> diag = {{x0, y0}, {x1, y1}};
    assert(navigation2d::PathCutsLiveOccupied(corner, diag, x0, y0, .14));
    assert(navigation2d::PathCutsLiveOccupied(corner, diag, x0, y0, .10));
  }
}
