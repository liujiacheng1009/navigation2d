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
  // Wide free gap into a deep unknown room: both free-run axes hit the probe
  // cap (same geometry as open-wall fringe), but unknown depth ≥0.55 m must
  // still count as a corridor mouth so unfinished rooms stay enterable.
  {
    navigation2d::ExplorationGrid wide_room;
    wide_room.width = 50;
    wide_room.height = 50;
    wide_room.resolution = .05;
    wide_room.cells.assign(static_cast<std::size_t>(wide_room.width * wide_room.height), 100);
    for (int row = 4; row < 28; ++row) {
      for (int col = 4; col < 46; ++col) {
        wide_room.cells[static_cast<std::size_t>(row * wide_room.width + col)] = 0;
      }
    }
    // Deep unknown room north of a wide free edge.
    for (int row = 28; row < 46; ++row) {
      for (int col = 10; col < 40; ++col) {
        wide_room.cells[static_cast<std::size_t>(row * wide_room.width + col)] = -1;
      }
    }
    navigation2d::FrontierExplorerConfig wcfg;
    wcfg.footprint_clearance = navigation2d::kPlanningRadius();
    wcfg.minimum_frontier_cells = 4;
    wcfg.required_frontier_observations = 1;
    navigation2d::FrontierExplorer wex(wcfg);
    wex.UpdateMap(wide_room);
    const double wfx = (25 + .5) * wide_room.resolution;
    const double wfy = (27 + .5) * wide_room.resolution;
    assert(wex.IsCorridorMouth(wfx, wfy));
  }
}
