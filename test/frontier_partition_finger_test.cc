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
  // Alcove partition finger: thin occupied fin must stay lethal to PathCuts.
  {
    navigation2d::ExplorationGrid finger;
    finger.width = 60;
    finger.height = 48;
    finger.resolution = .05;
    finger.origin_x = 0.;
    finger.origin_y = 0.;
    finger.cells.assign(static_cast<std::size_t>(finger.width * finger.height), 100);
    const auto fp = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row) {
        for (int col = col0; col < col1; ++col) {
          finger.cells[static_cast<std::size_t>(row * finger.width + col)] = value;
        }
      }
    };
    fp(2, 58, 4, 36, 0);      // open room
    fp(2, 58, 36, 44, 100);   // north wall mass
    fp(28, 31, 20, 36, 100);  // 0.15×0.80 m fin hanging south from wall
    const double x0 = (20 + .5) * finger.resolution;
    const double y0 = (28 + .5) * finger.resolution;
    const double x1 = (38 + .5) * finger.resolution;
    const double y1 = (28 + .5) * finger.resolution;
    const std::vector<std::pair<double, double>> through_fin = {{x0, y0}, {x1, y1}};
    assert(navigation2d::PathCutsLiveOccupied(finger, through_fin, x0, y0,
                                              navigation2d::kPlanningRadius()));
    assert(finger.cells[28 * finger.width + 29] == 100);
    assert(finger.cells[21 * finger.width + 29] == 100);
  }
}
