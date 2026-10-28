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
  // Two doorways whose mouths are ~2.0 m apart (beyond the 1.5 m mouth merge).
  // Observing one cools that mouth only.
  navigation2d::FrontierExplorerConfig pair_config;
  pair_config.minimum_frontier_cells = 6;
  pair_config.footprint_clearance = navigation2d::kPlanningRadius();
  pair_config.minimum_standoff = .10;
  pair_config.maximum_standoff = 1.10;
  pair_config.blacklist_radius = 1.20;
  pair_config.required_frontier_observations = 1;
  navigation2d::ExplorationGrid pair;
  pair.width = 60;
  pair.height = 64;
  pair.resolution = .05;
  pair.cells.assign(static_cast<std::size_t>(pair.width * pair.height), 100);
  const auto pair_paint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row) {
      for (int col = col0; col < col1; ++col) {
        pair.cells[static_cast<std::size_t>(row * pair.width + col)] = value;
      }
    }
  };
  pair_paint(2, 22, 2, 62, 0);
  pair_paint(22, 50, 6, 16, -1);
  pair_paint(22, 50, 46, 56, -1);
  navigation2d::FrontierExplorer pair_explorer(pair_config);
  pair_explorer.UpdateMap(pair);
  const auto pair_goals =
      pair_explorer.SelectGoals((8 + .5) * pair.resolution, (11 + .5) * pair.resolution);
  const navigation2d::ExplorationGoal* lower = nullptr;
  const navigation2d::ExplorationGoal* upper = nullptr;
  for (const auto& goal : pair_goals) {
    if (goal.frontier_y < (32 + .5) * pair.resolution) {
      if (lower == nullptr || goal.frontier_cells > lower->frontier_cells) {
        lower = &goal;
      }
    } else if (upper == nullptr || goal.frontier_cells > upper->frontier_cells) {
      upper = &goal;
    }
  }
  assert(lower != nullptr && upper != nullptr);
  assert(std::abs(lower->frontier_y - upper->frontier_y) > 1.5);
  assert(std::abs(lower->frontier_y - upper->frontier_y) < 2.5);
  pair_explorer.RecordAttempt(*lower, true);
  const auto upper_goals =
      pair_explorer.SelectGoals((8 + .5) * pair.resolution, (51 + .5) * pair.resolution);
  bool upper_remains = false;
  for (const auto& goal : upper_goals) {
    if (std::hypot(goal.frontier_x - upper->frontier_x, goal.frontier_y - upper->frontier_y) <
        .40) {
      upper_remains = true;
    }
  }
  assert(upper_remains);
  // A failed standoff must not suppress the entire boundary or its adjacent
  // doorway. The old span+0.25 radius could cover both rooms at once.
  navigation2d::FrontierExplorer failed_pair(pair_config);
  failed_pair.UpdateMap(pair);
  auto failed = *lower;
  failed.x -= 1.5;
  failed_pair.RecordAttempt(failed, false);
  const auto alternatives =
      failed_pair.SelectGoals((8 + .5) * pair.resolution, (11 + .5) * pair.resolution);
  bool lower_available = false, upper_available = false;
  for (const auto& goal : alternatives) {
    lower_available |=
        std::hypot(goal.frontier_x - lower->frontier_x, goal.frontier_y - lower->frontier_y) < .4;
    upper_available |=
        std::hypot(goal.frontier_x - upper->frontier_x, goal.frontier_y - upper->frontier_y) < .4;
  }
  assert(lower_available && upper_available);
}
