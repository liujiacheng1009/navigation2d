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
  // One unknown cell in an open room is below minimum_frontier_cells, so it
  // stays out of the exploring tour. It must still become an approach goal.
  // The robot reaches it through ordinary open space; it does not need a
  // narrow-corridor fallback. Three adjacent unknown cells expand into a
  // 7-cell frontier and already produced a normal goal before this change.
  navigation2d::FrontierExplorerConfig leftover_config;
  leftover_config.minimum_frontier_cells = 6;
  leftover_config.footprint_clearance = .40;
  leftover_config.minimum_standoff = .55;
  leftover_config.maximum_standoff = 1.10;
  leftover_config.required_frontier_observations = 1;
  // Mature map (>= 10000 known cells): a 1-cell speck is not a doorway and
  // must stay out of SelectGoals, while SelectLeftoverApproach still finds it.
  navigation2d::ExplorationGrid leftover;
  leftover.width = leftover.height = 110;
  leftover.resolution = .1;
  leftover.cells.assign(static_cast<std::size_t>(leftover.width * leftover.height), 100);
  for (int row = 2; row < 108; ++row) {
    for (int col = 2; col < 108; ++col) {
      leftover.cells[row * leftover.width + col] = 0;
    }
  }
  leftover.cells[100 * leftover.width + 100] = -1;
  navigation2d::FrontierExplorer leftover_explorer(leftover_config);
  leftover_explorer.UpdateMap(leftover);
  assert(leftover_explorer.KnownCells() >= 10000);
  assert(leftover_explorer.SelectGoals(5., 5.).empty());
  const auto leftover_goal = leftover_explorer.SelectLeftoverApproach(5., 5.);
  assert(leftover_goal.has_value());
  assert(leftover_goal->leftover_approach);
  assert(leftover_explorer.raw_frontier_cells() > 0);
  // A temporary failure cool must not hide the unfinished tip from leftover
  // completing — that was returning PARTIAL with white alcoves still visible.
  leftover_explorer.RecordAttempt(*leftover_goal, false);
  assert(leftover_explorer.SelectGoals(5., 5.).empty());
  const auto leftover_after_fail = leftover_explorer.SelectLeftoverApproach(5., 5.);
  assert(leftover_after_fail.has_value());
  leftover_explorer.ClearFailedBlacklist();
  assert(leftover_explorer.SelectLeftoverApproaches(5., 5., 3).size() >= 1);
  // Wide pocket cool (tip-leave) must suppress leftover re-entry into a wall
  // tip — narrow failure cools above must not.
  leftover_explorer.CoolRegion(leftover_goal->frontier_x, leftover_goal->frontier_y, 1.80, 3000u);
  assert(!leftover_explorer.SelectLeftoverApproach(5., 5.).has_value());
}
