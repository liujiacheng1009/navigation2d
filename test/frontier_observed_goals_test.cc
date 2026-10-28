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
  // Tour goals must stay on observed free — unknown exterior must not become
  // a planning GOAL outside the mapped room.
  {
    navigation2d::FrontierExplorerConfig flood_config;
    flood_config.minimum_frontier_cells = 4;
    flood_config.footprint_clearance = .30;
    flood_config.minimum_standoff = .20;
    flood_config.maximum_standoff = 1.20;
    flood_config.required_frontier_observations = 1;
    navigation2d::ExplorationGrid flooded;
    flooded.width = 60;
    flooded.height = 40;
    flooded.resolution = .05;
    flooded.cells.assign(static_cast<std::size_t>(flooded.width * flooded.height), 100);
    for (int row = 8; row < 32; ++row) {
      for (int col = 8; col < 36; ++col) {
        flooded.cells[static_cast<std::size_t>(row * flooded.width + col)] = 0;
      }
    }
    // Narrow free nick in the east wall, then a band of unknown outside.
    for (int row = 18; row < 22; ++row) {
      flooded.cells[static_cast<std::size_t>(row * flooded.width + 36)] = 0;
    }
    for (int row = 10; row < 30; ++row) {
      for (int col = 37; col < 55; ++col) {
        flooded.cells[static_cast<std::size_t>(row * flooded.width + col)] = -1;
      }
    }
    navigation2d::FrontierExplorer flood_explorer(flood_config);
    flood_explorer.UpdateMap(flooded);
    const double fx = (20 + .5) * flooded.resolution;
    const double fy = (20 + .5) * flooded.resolution;
    const auto flood_goals = flood_explorer.SelectGoals(fx, fy);
    for (const auto& goal : flood_goals) {
      const int gcol =
          static_cast<int>(std::floor((goal.x - flooded.origin_x) / flooded.resolution));
      const int grow =
          static_cast<int>(std::floor((goal.y - flooded.origin_y) / flooded.resolution));
      assert(gcol >= 0 && grow >= 0 && gcol < flooded.width && grow < flooded.height);
      assert(flooded.cells[static_cast<std::size_t>(grow * flooded.width + gcol)] == 0);
    }
  }
}
