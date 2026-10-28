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
  // Continuous free↔unknown exterior wall: PathCuts rejects through-wall path.
  {
    navigation2d::ExplorationGrid exterior;
    exterior.width = 60;
    exterior.height = 40;
    exterior.resolution = .05;
    exterior.origin_x = 0.;
    exterior.origin_y = 0.;
    exterior.cells.assign(static_cast<std::size_t>(exterior.width * exterior.height), 100);
    const auto epaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row) {
        for (int col = col0; col < col1; ++col) {
          exterior.cells[static_cast<std::size_t>(row * exterior.width + col)] = value;
        }
      }
    };
    epaint(2, 58, 4, 18, 0);     // explored hall
    epaint(2, 58, 18, 19, 100);  // long structural wall (no door gap)
    epaint(2, 58, 19, 36, -1);   // unfinished room unknown
    const double rx = (30 + .5) * exterior.resolution;
    const double ry = (16 + .5) * exterior.resolution;
    const std::vector<std::pair<double, double>> through = {
        {rx, ry},
        {rx, (24 + .5) * exterior.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(exterior, through, rx, ry));
    assert(exterior.cells[18 * exterior.width + 30] == 100);
  }
}
