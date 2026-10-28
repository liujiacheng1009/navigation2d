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
  // Production open-room config: viewpoints must stay off the wall rim.
  // Mapping observes frontiers from mid-room; min standoff 0.45 m.
  {
    navigation2d::FrontierExplorerConfig open_config;
    open_config.minimum_frontier_cells = 6;
    open_config.footprint_clearance = .38;
    open_config.minimum_standoff = .45;
    open_config.maximum_standoff = 1.20;
    open_config.required_frontier_observations = 1;
    navigation2d::ExplorationGrid open_room;
    open_room.width = 80;
    open_room.height = 60;
    open_room.resolution = .05;
    open_room.cells.assign(static_cast<std::size_t>(open_room.width * open_room.height), 100);
    for (int row = 8; row < 52; ++row) {
      for (int col = 8; col < 72; ++col) {
        open_room.cells[static_cast<std::size_t>(row * open_room.width + col)] = 0;
      }
    }
    // Unknown strip along the far wall — open-boundary frontier.
    for (int row = 8; row < 52; ++row) {
      for (int col = 72; col < 78; ++col) {
        open_room.cells[static_cast<std::size_t>(row * open_room.width + col)] = -1;
      }
    }
    navigation2d::FrontierExplorer open_explorer(open_config);
    open_explorer.UpdateMap(open_room);
    const double ox = (20 + .5) * open_room.resolution;
    const double oy = (30 + .5) * open_room.resolution;
    const auto open_goals = open_explorer.SelectGoals(ox, oy);
    assert(!open_goals.empty());
    for (const auto& goal : open_goals) {
      const double standoff = std::hypot(goal.x - goal.frontier_x, goal.y - goal.frontier_y);
      assert(standoff + 1e-9 >= open_config.minimum_standoff);
      double nearest_occupied = 10.;
      for (int row = 0; row < open_room.height; ++row) {
        for (int col = 0; col < open_room.width; ++col) {
          if (open_room.cells[static_cast<std::size_t>(row * open_room.width + col)] <= 0) {
            continue;
          }
          const double x = (col + .5) * open_room.resolution;
          const double y = (row + .5) * open_room.resolution;
          nearest_occupied = std::min(nearest_occupied, std::hypot(goal.x - x, goal.y - y));
        }
      }
      assert(nearest_occupied + 1e-9 >= open_config.footprint_clearance);
    }
  }
}
