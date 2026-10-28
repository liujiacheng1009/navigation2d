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
  auto tour_config = navigation2d::test::DoorFrontierConfig();
  // Pinched alcove tip: frontier cells fail ClearAt while a centerline pose
  // behind them is still on the discovery graph. Those tips must become
  // candidates (approachable frontier), not vanish into candidates=0.
  navigation2d::ExplorationGrid tip;
  tip.width = 50;
  tip.height = 60;
  tip.resolution = .05;
  tip.cells.assign(static_cast<std::size_t>(tip.width * tip.height), 100);
  const auto tpaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row) {
      for (int col = col0; col < col1; ++col) {
        tip.cells[static_cast<std::size_t>(row * tip.width + col)] = value;
      }
    }
  };
  tpaint(8, 42, 4, 30, 0);     // open hall
  tpaint(19, 29, 30, 48, 0);   // 0.50 m north alcove (planning radius .22)
  tpaint(19, 29, 48, 56, -1);  // unfinished tip unknown
  // Thicken the tip free cells with near walls so ClearAt(.22) fails on the
  // frontier row itself but the approach row still clears.
  tpaint(19, 20, 46, 48, 100);
  tpaint(28, 29, 46, 48, 100);
  navigation2d::FrontierExplorerConfig tip_config = tour_config;
  tip_config.footprint_clearance = .30;
  tip_config.required_frontier_observations = 1;
  navigation2d::FrontierExplorer tip_explorer(tip_config);
  tip_explorer.UpdateMap(tip);
  const auto tip_goals =
      tip_explorer.SelectGoals((25 + .5) * tip.resolution, (15 + .5) * tip.resolution);
  assert(!tip_goals.empty());
  assert(tip_explorer.candidate_frontier_goals() > 0);
  assert(tip_goals.front().frontier_y > 1.8);
}
