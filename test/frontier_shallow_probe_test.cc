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
  // Easy shallow doorway: free gap ≥0.50 m but room only ~0.40 m deep.
  // Fixed 0.55 m entry minimum used to return nullopt and abandon the mouth
  // while the chassis sat on a clearly passable threshold (UI red box).
  {
    navigation2d::ExplorationGrid shallow;
    shallow.width = 50;
    shallow.height = 40;
    shallow.resolution = .05;
    shallow.origin_x = 0.;
    shallow.origin_y = 0.;
    shallow.cells.assign(static_cast<std::size_t>(shallow.width * shallow.height), 100);
    const auto sp = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row) {
        for (int col = col0; col < col1; ++col) {
          shallow.cells[static_cast<std::size_t>(row * shallow.width + col)] = value;
        }
      }
    };
    sp(2, 48, 4, 22, 0);     // hall
    sp(18, 32, 22, 30, 0);   // 0.70 m × 0.40 m shallow room (free)
    sp(18, 32, 30, 36, -1);  // unknown past the pocket
    navigation2d::FrontierExplorerConfig scfg;
    scfg.footprint_clearance = navigation2d::kPlanningRadius();
    scfg.minimum_frontier_cells = 4;
    scfg.minimum_standoff = .10;
    scfg.maximum_standoff = 1.10;
    scfg.required_frontier_observations = 1;
    navigation2d::FrontierExplorer sexplorer(scfg);
    sexplorer.UpdateMap(shallow);
    navigation2d::ExplorationGoal mouth;
    mouth.x = (25 + .5) * shallow.resolution;
    mouth.y = (20 + .5) * shallow.resolution;
    mouth.frontier_x = (25 + .5) * shallow.resolution;
    mouth.frontier_y = (22 + .5) * shallow.resolution;
    mouth.yaw = 1.57;
    const double gap = sexplorer.MouthGapWidth(mouth.frontier_x, mouth.frontier_y);
    assert(gap + 1e-9 >= .50);
    const double free_depth = sexplorer.MouthFreeDepth(mouth, mouth.x, mouth.y - .3);
    assert(free_depth + 1e-9 >= .30);
    // Deep schedule (old StartEntryProbe) must fail; shallow schedule works.
    assert(!sexplorer.SelectEntryProbe(mouth, mouth.x, mouth.y - .3, .55, 1.30));
    const auto poke = sexplorer.SelectEntryProbe(mouth, mouth.x, mouth.y - .3, .22, .55);
    assert(poke.has_value());
    assert(poke->y > mouth.frontier_y - 1e-6);
    // Mid-gap: not glued to either jamb of the 0.70 m mouth.
    assert(poke->x > (18 + .5) * shallow.resolution + .14);
    assert(poke->x < (32 + .5) * shallow.resolution - .14);
  }
}
