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
  // Entry-style unknown paint must not turn a long structural bar into a
  // PathCuts-legal chord (UI GLOBAL 穿墙). Paint free past the wall, then
  // PathCuts on the painted grid must still reject.
  {
    navigation2d::ExplorationGrid painted_wall;
    painted_wall.width = 40;
    painted_wall.height = 40;
    painted_wall.resolution = .05;
    painted_wall.origin_x = 0.;
    painted_wall.origin_y = 0.;
    painted_wall.cells.assign(static_cast<std::size_t>(painted_wall.width * painted_wall.height),
                              100);
    const auto pp = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row) {
        for (int col = col0; col < col1; ++col) {
          painted_wall.cells[static_cast<std::size_t>(row * painted_wall.width + col)] = value;
        }
      }
    };
    pp(4, 36, 4, 20, 0);     // hall
    pp(4, 36, 20, 21, 100);  // full-width structural bar
    pp(4, 36, 21, 36, -1);   // unknown room
    // Simulate the old PathCuts entry paint: unknown → free in a strip.
    const int mid = 20;
    for (int row = 21; row < 30; ++row) {
      for (int col = mid - 2; col <= mid + 2; ++col) {
        painted_wall.cells[static_cast<std::size_t>(row * painted_wall.width + col)] = 0;
      }
    }
    const double rx = (mid + .5) * painted_wall.resolution;
    const double ry = (18 + .5) * painted_wall.resolution;
    const std::vector<std::pair<double, double>> through = {
        {rx, ry},
        {rx, (24 + .5) * painted_wall.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(painted_wall, through, rx, ry));
  }
}
