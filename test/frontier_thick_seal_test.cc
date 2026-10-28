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
  // Free↔free SLAM seal 0.30 m thick: PathCuts rejects the through-seal chord.
  {
    navigation2d::ExplorationGrid thick_seal;
    thick_seal.width = 60;
    thick_seal.height = 40;
    thick_seal.resolution = .05;
    thick_seal.origin_x = 0.;
    thick_seal.origin_y = 0.;
    thick_seal.cells.assign(static_cast<std::size_t>(thick_seal.width * thick_seal.height), 100);
    const auto tp = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row) {
        for (int col = col0; col < col1; ++col) {
          thick_seal.cells[static_cast<std::size_t>(row * thick_seal.width + col)] = value;
        }
      }
    };
    tp(2, 58, 4, 18, 0);      // hall
    tp(2, 58, 24, 36, 0);     // room (known free)
    tp(22, 34, 18, 24, 100);  // 0.30 m × 0.60 m occupied door seal
    const double rx = (28 + .5) * thick_seal.resolution;
    const double ry = (17 + .5) * thick_seal.resolution;
    const std::vector<std::pair<double, double>> entry = {
        {rx, ry},
        {rx, (26 + .5) * thick_seal.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(thick_seal, entry, rx, ry));
  }
}
