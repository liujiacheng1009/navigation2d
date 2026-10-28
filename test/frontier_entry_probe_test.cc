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
  auto door_config = navigation2d::test::DoorFrontierConfig();
  // A doorway arrival is only the observation-side pose. When a partially
  // observed narrow opening is safely opened in the private planning grid,
  // the explorer must be able to produce a reachable pose on the far side of
  // the frontier for the bounded entry probe.
  navigation2d::ExplorationGrid entry;
  entry.width = 50;
  entry.height = 30;
  entry.resolution = .05;
  entry.cells.assign(static_cast<std::size_t>(entry.width * entry.height), 100);
  for (int row = 4; row < 26; ++row) {
    for (int col = 2; col < 16; ++col) {
      entry.cells[static_cast<std::size_t>(row * entry.width + col)] = 0;
    }
  }
  for (int row = 4; row < 26; ++row) {
    for (int col = 21; col < 29; ++col) {
      entry.cells[static_cast<std::size_t>(row * entry.width + col)] = 0;
    }
  }
  for (int row = 8; row < 22; ++row) {
    for (int col = 16; col < 21; ++col) {
      entry.cells[static_cast<std::size_t>(row * entry.width + col)] = -1;
    }
  }
  navigation2d::FrontierExplorer entry_explorer(door_config);
  entry_explorer.UpdateMap(entry);
  // The opening is reachable in the private planning graph, while it is
  // still unknown in the observation map; completion must account for it.
  assert(entry_explorer.ReachableUnknownCells(.4, .7) > 0);
  const auto entry_goals = entry_explorer.SelectGoals(.4, .7);
  assert(!entry_goals.empty());
  // OneGoalPerMouth prefers an entry-side pose when the planning graph opened
  // the gap: the task point must lie past the frontier toward the room.
  const auto& entry_goal = entry_goals.front();
  const double entry_depth =
      ((entry_goal.x - entry_goal.frontier_x) * (entry_goal.frontier_x - .4) +
       (entry_goal.y - entry_goal.frontier_y) * (entry_goal.frontier_y - .7)) /
      std::hypot(entry_goal.frontier_x - .4, entry_goal.frontier_y - .7);
  assert(entry_depth >= .15);
  // SelectEntryProbe still works from a hallway-side standoff for the
  // runtime entry state machine.
  navigation2d::ExplorationGoal hallway_standoff = entry_goal;
  hallway_standoff.x = entry_goal.frontier_x - .45;
  hallway_standoff.y = entry_goal.frontier_y;
  const auto entry_probe = entry_explorer.SelectEntryProbe(hallway_standoff, .4, .7, .20, .70);
  assert(entry_probe.has_value());
  const double entry_dx = entry_probe->x - hallway_standoff.frontier_x;
  const double entry_dy = entry_probe->y - hallway_standoff.frontier_y;
  const double entry_direction_x = hallway_standoff.frontier_x - hallway_standoff.x;
  const double entry_direction_y = hallway_standoff.frontier_y - hallway_standoff.y;
  const double entry_direction_length = std::hypot(entry_direction_x, entry_direction_y);
  assert((entry_dx * entry_direction_x + entry_dy * entry_direction_y) / entry_direction_length >=
         .20);
  // Probe may sit on planning free past the frontier when the throat is open.
  const int probe_col =
      static_cast<int>(std::floor((entry_probe->x - entry.origin_x) / entry.resolution));
  const int probe_row =
      static_cast<int>(std::floor((entry_probe->y - entry.origin_y) / entry.resolution));
  const auto& observed = entry;
  const bool probe_raw_free =
      probe_col >= 0 && probe_row >= 0 && probe_col < observed.width &&
      probe_row < observed.height &&
      observed.cells[static_cast<std::size_t>(probe_row) * observed.width + probe_col] == 0;
  assert(!probe_raw_free || entry_probe->x > hallway_standoff.frontier_x - 1e-6);
}
