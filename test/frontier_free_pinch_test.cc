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
  // Free↔free pinch (0.25 m): without jamb carve the 0.22 m body clips the
  // posts, so PathCuts rejects the entry chord.
  {
    navigation2d::ExplorationGrid both_free;
    both_free.width = 60;
    both_free.height = 40;
    both_free.resolution = .05;
    both_free.origin_x = 0.;
    both_free.origin_y = 0.;
    both_free.cells.assign(static_cast<std::size_t>(both_free.width * both_free.height), 100);
    const auto bp = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row) {
        for (int col = col0; col < col1; ++col) {
          both_free.cells[static_cast<std::size_t>(row * both_free.width + col)] = value;
        }
      }
    };
    bp(2, 58, 4, 18, 0);    // hall (known free)
    bp(2, 58, 22, 36, 0);   // room (known free)
    bp(27, 32, 18, 22, 0);  // 0.25 m free throat (5 cells)
    const double rx = (29 + .5) * both_free.resolution;
    const double ry = (17 + .5) * both_free.resolution;
    const std::vector<std::pair<double, double>> entry = {
        {rx, ry},
        {rx, (24 + .5) * both_free.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(both_free, entry, rx, ry));
  }
}
