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
  // Occupied door-width kitchen bar: PathCuts rejects until SLAM clears it.
  {
    navigation2d::ExplorationGrid sealed;
    sealed.width = 60;
    sealed.height = 40;
    sealed.resolution = .05;
    sealed.origin_x = 0.;
    sealed.origin_y = 0.;
    sealed.cells.assign(static_cast<std::size_t>(sealed.width * sealed.height), 100);
    const auto spaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row) {
        for (int col = col0; col < col1; ++col) {
          sealed.cells[static_cast<std::size_t>(row * sealed.width + col)] = value;
        }
      }
    };
    spaint(2, 58, 4, 18, 0);      // hall
    spaint(2, 58, 22, 36, -1);    // kitchen unknown
    spaint(20, 34, 18, 22, 0);    // 0.70 m mouth vestibule
    spaint(20, 34, 20, 21, 100);  // thin occupied bar
    const double rx = (27 + .5) * sealed.resolution;
    const double ry = (17 + .5) * sealed.resolution;
    const std::vector<std::pair<double, double>> entry = {
        {rx, ry},
        {rx, (24 + .5) * sealed.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(sealed, entry, rx, ry));
  }
}
