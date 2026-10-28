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
  // Junction: three nearby mouths + one far room. Tour must lead with a near
  // mouth; a far active goal must be preemptable until local mouths are gone.
  navigation2d::FrontierExplorerConfig junction_config;
  junction_config.minimum_frontier_cells = 6;
  junction_config.footprint_clearance = navigation2d::kPlanningRadius();
  junction_config.minimum_standoff = .10;
  junction_config.maximum_standoff = 1.10;
  junction_config.required_frontier_observations = 1;
  navigation2d::ExplorationGrid junction;
  junction.width = 120;
  junction.height = 70;
  junction.resolution = .05;
  junction.cells.assign(static_cast<std::size_t>(junction.width * junction.height), 100);
  const auto jpaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row) {
      for (int col = col0; col < col1; ++col) {
        junction.cells[static_cast<std::size_t>(row * junction.width + col)] = value;
      }
    }
  };
  jpaint(25, 70, 28, 42, 0);
  jpaint(36, 48, 8, 62, 0);
  jpaint(6, 25, 28, 42, -1);
  jpaint(36, 48, 62, 68, -1);
  jpaint(36, 48, 2, 8, -1);
  jpaint(100, 118, 28, 42, -1);
  navigation2d::FrontierExplorer junction_explorer(junction_config);
  junction_explorer.UpdateMap(junction);
  const double jx = (40 + .5) * junction.resolution;
  const double jy = (35 + .5) * junction.resolution;
  const auto junction_tour = junction_explorer.BuildTour(jx, jy);
  assert(!junction_tour.empty());
  assert(junction_tour.front().x < 4.5);
  navigation2d::ExplorationGoal far_active;
  far_active.x = (105 + .5) * junction.resolution;
  far_active.y = (35 + .5) * junction.resolution;
  far_active.frontier_x = (100 + .5) * junction.resolution;
  far_active.frontier_y = (35 + .5) * junction.resolution;
  far_active.frontier_cells = 20;
  far_active.information_gain = 10.;
  assert(junction_explorer.ShouldPreemptForNearerMouth(jx, jy, far_active, 6.0));
  jpaint(6, 25, 28, 42, 0);
  jpaint(36, 48, 62, 68, 0);
  jpaint(36, 48, 2, 8, 0);
  junction_explorer.UpdateMap(junction);
  assert(!junction_explorer.ShouldPreemptForNearerMouth(jx, jy, far_active, 6.0));
}
