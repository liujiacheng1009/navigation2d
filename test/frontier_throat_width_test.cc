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
  // Narrow free kitchen throat (0.40 m): without jamb carve the 0.22 m body
  // clips the posts, so PathCuts rejects. Widen to 0.70 m and the same chord
  // is legal.
  {
    navigation2d::ExplorationGrid pinch;
    pinch.width = 60;
    pinch.height = 40;
    pinch.resolution = .05;
    pinch.origin_x = 0.;
    pinch.origin_y = 0.;
    pinch.cells.assign(static_cast<std::size_t>(pinch.width * pinch.height), 100);
    const auto pp = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row) {
        for (int col = col0; col < col1; ++col) {
          pinch.cells[static_cast<std::size_t>(row * pinch.width + col)] = value;
        }
      }
    };
    pp(2, 58, 4, 18, 0);    // hall
    pp(2, 58, 22, 36, -1);  // kitchen unknown
    pp(24, 32, 18, 22, 0);  // 0.40 m free throat (8 cells)
    const double rx = (28 + .5) * pinch.resolution;
    const double ry = (17 + .5) * pinch.resolution;
    const std::vector<std::pair<double, double>> entry = {
        {rx, ry},
        {rx, (24 + .5) * pinch.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(pinch, entry, rx, ry));
    pp(20, 34, 18, 22, 0);  // widen to 0.70 m
    assert(!navigation2d::PathCutsLiveOccupied(pinch, entry, rx, ry));
  }
}
