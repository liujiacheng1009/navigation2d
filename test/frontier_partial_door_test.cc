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
  // The hall is wide. The doorway is only partly observed: a strip beside one
  // jamb is free and the rest of the gap, plus the room beyond, is unknown.
  // The existing corridor rule completes that gap so the viewpoint sits on
  // the centerline. The clearance stays 0.22 m. A narrower gap, and the room
  // past the doorway, stay unknown.
  navigation2d::FrontierExplorerConfig door_config;
  door_config.minimum_frontier_cells = 6;
  door_config.footprint_clearance = navigation2d::kPlanningRadius();
  door_config.minimum_standoff = .10;
  door_config.maximum_standoff = 1.10;
  door_config.required_frontier_observations = 1;
  navigation2d::ExplorationGrid door;
  door.width = 80;
  door.height = 56;
  door.resolution = .05;
  door.cells.assign(static_cast<std::size_t>(door.width * door.height), 100);
  const auto paint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row) {
      for (int col = col0; col < col1; ++col) {
        door.cells[static_cast<std::size_t>(row * door.width + col)] = value;
      }
    }
  };
  paint(2, 28, 4, 48, 0);
  paint(28, 48, 18, 32, -1);
  paint(28, 29, 18, 22, 0);
  paint(48, 74, 4, 48, -1);
  // 0.30 m from wall to wall, below the corridor the chassis can enter.
  paint(60, 63, 50, 54, 0);
  paint(63, 66, 50, 54, -1);
  // Without OpenPassable flood, unknown beyond the free speck stays unknown.
  assert(door.cells[25 * door.width + 36] == -1);
  assert(door.cells[25 * door.width + 48] == -1);
  assert(door.cells[52 * door.width + 64] == -1);
  navigation2d::FrontierExplorer door_explorer(door_config);
  door_explorer.UpdateMap(door);
  const auto door_goals =
      door_explorer.SelectGoals((8 + .5) * door.resolution, (25 + .5) * door.resolution);
  assert(!door_goals.empty());
  const double door_mouth = (28 + .5) * door.resolution;
  bool on_centerline = false;
  for (const auto& goal : door_goals) {
    if (goal.x < door_mouth) {
      continue;
    }
    on_centerline = true;
    assert(std::hypot(goal.x - goal.frontier_x, goal.y - goal.frontier_y) <=
           door_config.maximum_standoff);
    double nearest_occupied = 1.;
    for (int row = 0; row < door.height; ++row) {
      for (int col = 0; col < door.width; ++col) {
        if (door.cells[static_cast<std::size_t>(row * door.width + col)] <= 0) {
          continue;
        }
        const double x = (col + .5) * door.resolution;
        const double y = (row + .5) * door.resolution;
        nearest_occupied = std::min(nearest_occupied, std::hypot(goal.x - x, goal.y - y));
      }
    }
    assert(nearest_occupied >= navigation2d::kPlanningRadius());
  }
  assert(on_centerline);
}
