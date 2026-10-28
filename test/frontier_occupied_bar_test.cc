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
  // Door-width occupied bar across a kitchen mouth: SLAM occupied is lethal.
  // PathCuts rejects the entry chord until the map clears the seal.
  navigation2d::ExplorationGrid barred;
  barred.width = 60;
  barred.height = 40;
  barred.resolution = .05;
  barred.cells.assign(static_cast<std::size_t>(barred.width * barred.height), 100);
  const auto bpaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row) {
      for (int col = col0; col < col1; ++col) {
        barred.cells[static_cast<std::size_t>(row * barred.width + col)] = value;
      }
    }
  };
  bpaint(2, 58, 4, 18, 0);      // south hall free
  bpaint(2, 58, 22, 36, -1);    // north kitchen unknown
  bpaint(20, 34, 18, 22, 0);    // 0.70 m mouth
  bpaint(20, 34, 20, 21, 100);  // 1-cell occupied bar sealing the mouth
  {
    const double rx = (27 + .5) * barred.resolution;
    const double ry = (17 + .5) * barred.resolution;
    const std::vector<std::pair<double, double>> entry = {
        {rx, ry},
        {rx, (24 + .5) * barred.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(barred, entry, rx, ry));
  }
  assert(barred.cells[20 * barred.width + 27] == 100);
  assert(barred.cells[24 * barred.width + 27] == -1);
}
