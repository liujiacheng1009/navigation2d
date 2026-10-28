#include "navigation2d/exploration/frontier_explorer.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "navigation2d/test/frontier_test_scenes.h"

int main() {
  navigation2d::FrontierExplorerConfig config;
  config.minimum_frontier_cells = 3;
  config.footprint_clearance = .1;
  config.required_frontier_observations = 1;
  navigation2d::FrontierExplorer explorer(config);
  navigation2d::ExplorationGrid grid;
  grid.width = grid.height = 20;
  grid.resolution = .1;
  grid.cells.assign(400, -1);
  for (int row = 4; row < 16; ++row) {
    for (int col = 4; col < 16; ++col) {
      grid.cells[row * grid.width + col] = 0;
    }
  }
  explorer.UpdateMap(grid);
  const auto goals = explorer.SelectGoals(1., 1.);
  assert(!goals.empty());
  assert(explorer.GoalStillFrontier(goals.front()));
  const int frontier_col =
      static_cast<int>((goals.front().frontier_x - grid.origin_x) / grid.resolution);
  const int frontier_row =
      static_cast<int>((goals.front().frontier_y - grid.origin_y) / grid.resolution);
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      const int col = frontier_col + dx, row = frontier_row + dy;
      if (col >= 0 && row >= 0 && col < grid.width && row < grid.height) {
        grid.cells[row * grid.width + col] = 0;
      }
    }
  }
  explorer.UpdateMap(grid);
  assert(!explorer.GoalStillFrontier(goals.front()));
  explorer.RecordAttempt(goals.front(), false);
  assert(explorer.failed_goals() == 1);
  const auto remaining = explorer.SelectGoals(1., 1.);
  for (const auto& goal : remaining) {
    assert(goal.frontier_x != goals.front().frontier_x ||
           goal.frontier_y != goals.front().frontier_y);
  }
  assert(!explorer.CompletionEligible(explorer.KnownCells()));
}
