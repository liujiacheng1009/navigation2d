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
  // Physical 0.70 m kitchen door with production clearance (.30). Viewpoint
  // nomination must still place a task past the mouth into the kitchen; the
  // old corridor ClearAt(.30) rejected the centerline after jamb rasterization.
  navigation2d::FrontierExplorerConfig kitchen_config;
  kitchen_config.minimum_frontier_cells = 6;
  kitchen_config.footprint_clearance = .30;
  kitchen_config.minimum_standoff = .10;
  kitchen_config.maximum_standoff = 1.10;
  kitchen_config.required_frontier_observations = 1;
  navigation2d::ExplorationGrid kitchen;
  kitchen.width = 80;
  kitchen.height = 40;
  kitchen.resolution = .05;
  kitchen.cells.assign(static_cast<std::size_t>(kitchen.width * kitchen.height), 100);
  const auto kpaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row) {
      for (int col = col0; col < col1; ++col) {
        kitchen.cells[static_cast<std::size_t>(row * kitchen.width + col)] = value;
      }
    }
  };
  // Hall on the left; 0.70 m = 14-cell vertical doorway; kitchen unknown beyond.
  // the gap as a partly observed passable opening (same contract as door_test).
  kpaint(2, 28, 4, 36, 0);
  kpaint(28, 48, 13, 27, -1);  // 14 cells * 0.05 = 0.70 m
  kpaint(28, 30, 13, 20, 0);
  kpaint(48, 74, 4, 36, -1);
  assert(kitchen.cells[20 * kitchen.width + 36] == 0);
  navigation2d::FrontierExplorer kitchen_explorer(kitchen_config);
  kitchen_explorer.UpdateMap(kitchen);
  const double kx = (10 + .5) * kitchen.resolution;
  const double ky = (20 + .5) * kitchen.resolution;
  assert(kitchen_explorer.ReachableUnknownCells(kx, ky) > 0);
  const auto kitchen_goals = kitchen_explorer.SelectGoals(kx, ky);
  assert(!kitchen_goals.empty());
  const auto& kg = kitchen_goals.front();
  const double kitchen_depth = ((kg.x - kg.frontier_x) * (kg.frontier_x - kx) +
                                (kg.y - kg.frontier_y) * (kg.frontier_y - ky)) /
                               std::hypot(kg.frontier_x - kx, kg.frontier_y - ky);
  assert(kitchen_depth >= .10);
  navigation2d::ExplorationGoal hall_standoff = kg;
  hall_standoff.x = kg.frontier_x - .45;
  hall_standoff.y = kg.frontier_y;
  const auto kitchen_probe = kitchen_explorer.SelectEntryProbe(hall_standoff, kx, ky, .20, .70);
  assert(kitchen_probe.has_value());
  assert(kitchen_probe->x > kg.frontier_x - .05);
  // Doorway probe / standoff must sit mid-gap, not glued to a jamb.
  {
    const int pcol =
        static_cast<int>(std::floor((kitchen_probe->x - kitchen.origin_x) / kitchen.resolution));
    const int prow =
        static_cast<int>(std::floor((kitchen_probe->y - kitchen.origin_y) / kitchen.resolution));
    double nearest = 10.;
    for (int row = 0; row < kitchen.height; ++row) {
      for (int col = 0; col < kitchen.width; ++col) {
        if (kitchen.cells[static_cast<std::size_t>(row * kitchen.width + col)] <= 0) {
          continue;
        }
        const double ox = (col + .5) * kitchen.resolution;
        const double oy = (row + .5) * kitchen.resolution;
        nearest = std::min(nearest, std::hypot(kitchen_probe->x - ox, kitchen_probe->y - oy));
      }
    }
    assert(nearest + 1e-9 >= .14);
    (void)pcol;
    (void)prow;
  }
  // Probe must sit on observed free, not flooded unknown past the map.
  {
    const int pc = static_cast<int>(std::floor(kitchen_probe->x / kitchen.resolution));
    const int pr = static_cast<int>(std::floor(kitchen_probe->y / kitchen.resolution));
    assert(kitchen.cells[static_cast<std::size_t>(pr) * kitchen.width + pc] == 0);
  }
  // Lateral hall standoff (beside the jamb): old viewpoint→frontier axis
  // walked along the wall; unknown-centroid must still send the probe into
  // the kitchen (+x).
  navigation2d::ExplorationGoal lateral = kg;
  lateral.x = kg.frontier_x;
  lateral.y = kg.frontier_y - .55;
  const auto lateral_probe = kitchen_explorer.SelectEntryProbe(lateral, kx, ky, .20, .70);
  assert(lateral_probe.has_value());
  assert(lateral_probe->x > kg.frontier_x + .10);
}
