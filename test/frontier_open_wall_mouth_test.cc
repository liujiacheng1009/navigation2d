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
  // Open-wall frontier expansion is not a corridor mouth even when the free
  // span reads large (live gap=1.65 m along a wall → false "easy door").
  {
    navigation2d::ExplorationGrid open_wall;
    open_wall.width = 60;
    open_wall.height = 40;
    open_wall.resolution = .05;
    open_wall.cells.assign(static_cast<std::size_t>(open_wall.width * open_wall.height), 100);
    for (int row = 4; row < 36; ++row) {
      for (int col = 4; col < 56; ++col) {
        open_wall.cells[static_cast<std::size_t>(row * open_wall.width + col)] = 0;
      }
    }
    // North unknown edge — open expansion, not a door throat.
    for (int col = 4; col < 56; ++col) {
      open_wall.cells[static_cast<std::size_t>(35 * open_wall.width + col)] = -1;
    }
    navigation2d::FrontierExplorerConfig ocfg;
    ocfg.footprint_clearance = navigation2d::kPlanningRadius();
    ocfg.minimum_frontier_cells = 4;
    ocfg.minimum_standoff = .10;
    ocfg.maximum_standoff = 1.10;
    ocfg.required_frontier_observations = 1;
    navigation2d::FrontierExplorer oex(ocfg);
    oex.UpdateMap(open_wall);
    const double fx = (30 + .5) * open_wall.resolution;
    const double fy = (34 + .5) * open_wall.resolution;
    assert(!oex.IsCorridorMouth(fx, fy));
    assert(oex.MouthGapWidth(fx, fy) > 1.40);
  }
}
