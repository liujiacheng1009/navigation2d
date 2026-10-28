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
  // free↔free door-width residual seal: PathCuts still rejects through-bar.
  {
    navigation2d::ExplorationGrid both_free;
    both_free.width = 60;
    both_free.height = 40;
    both_free.resolution = .05;
    both_free.cells.assign(static_cast<std::size_t>(both_free.width * both_free.height), 100);
    const auto fpaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row) {
        for (int col = col0; col < col1; ++col) {
          both_free.cells[static_cast<std::size_t>(row * both_free.width + col)] = value;
        }
      }
    };
    fpaint(2, 58, 4, 18, 0);      // south hall
    fpaint(2, 58, 22, 36, 0);     // north kitchen now explored
    fpaint(20, 34, 18, 22, 0);    // 0.70 m mouth vestibule
    fpaint(20, 34, 20, 21, 100);  // occupied bar
    fpaint(26, 29, 22, 24, -1);
    const double rx = (27 + .5) * both_free.resolution;
    const double ry = (17 + .5) * both_free.resolution;
    const std::vector<std::pair<double, double>> through_bar = {
        {rx, ry},
        {rx, (24 + .5) * both_free.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(both_free, through_bar, rx, ry));
  }
}
