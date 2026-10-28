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
  // Corridor island: chord through a solid pillar must reject.
  {
    navigation2d::ExplorationGrid island;
    island.width = 60;
    island.height = 40;
    island.resolution = .05;
    island.origin_x = 0.;
    island.origin_y = 0.;
    island.cells.assign(static_cast<std::size_t>(island.width * island.height), 100);
    const auto ip = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row) {
        for (int col = col0; col < col1; ++col) {
          island.cells[static_cast<std::size_t>(row * island.width + col)] = value;
        }
      }
    };
    ip(2, 58, 8, 32, 0);      // open hall
    ip(28, 34, 16, 24, 100);  // 0.30×0.40 m solid island
    const double x0 = (20 + .5) * island.resolution;
    const double y0 = (20 + .5) * island.resolution;
    const double x1 = (40 + .5) * island.resolution;
    const double y1 = (20 + .5) * island.resolution;
    const std::vector<std::pair<double, double>> through = {{x0, y0}, {x1, y1}};
    assert(navigation2d::PathCutsLiveOccupied(island, through, x0, y0,
                                              navigation2d::kPlanningRadius()));
    assert(island.cells[20 * island.width + 30] == 100);
  }
}
