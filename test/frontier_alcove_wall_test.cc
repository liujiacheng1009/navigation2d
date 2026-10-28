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
  // Structural free↔free wall with unknown past a short vestibule: PathCuts
  // must reject through-wall chords (no seal exemption).
  {
    navigation2d::ExplorationGrid alcove_wall;
    alcove_wall.width = 60;
    alcove_wall.height = 40;
    alcove_wall.resolution = .05;
    alcove_wall.origin_x = 0.;
    alcove_wall.origin_y = 0.;
    alcove_wall.cells.assign(static_cast<std::size_t>(alcove_wall.width * alcove_wall.height), 100);
    const auto apaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row) {
        for (int col = col0; col < col1; ++col) {
          alcove_wall.cells[static_cast<std::size_t>(row * alcove_wall.width + col)] = value;
        }
      }
    };
    apaint(2, 58, 4, 18, 0);     // south free
    apaint(2, 58, 18, 19, 100);  // continuous structural wall
    apaint(2, 58, 19, 22, 0);    // short free vestibule north of wall
    apaint(2, 58, 22, 36, -1);   // unfinished unknown within 4 cells of wall
    const double rx = (30 + .5) * alcove_wall.resolution;
    const double ry = (16 + .5) * alcove_wall.resolution;
    const std::vector<std::pair<double, double>> through = {
        {rx, ry},
        {rx, (24 + .5) * alcove_wall.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(alcove_wall, through, rx, ry));
  }
}
