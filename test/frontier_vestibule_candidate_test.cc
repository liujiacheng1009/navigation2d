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
  // Both-narrow vestibule (kitchen mouth): SelectGoals must still nominate a
  // candidate. The old XOR narrow_viewpoint treated both-narrow as open space
  // and left raw frontiers with zero candidates → early PARTIAL.
  navigation2d::ExplorationGrid vestibule;
  vestibule.width = 60;
  vestibule.height = 40;
  vestibule.resolution = .05;
  vestibule.cells.assign(static_cast<std::size_t>(vestibule.width * vestibule.height), 100);
  const auto vpaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row) {
      for (int col = col0; col < col1; ++col) {
        vestibule.cells[static_cast<std::size_t>(row * vestibule.width + col)] = value;
      }
    }
  };
  vpaint(4, 28, 14, 26, 0);    // hall
  vpaint(28, 40, 15, 25, 0);   // 0.60 m × 0.50 m vestibule (both-narrow)
  vpaint(40, 56, 10, 30, -1);  // room unknown
  navigation2d::FrontierExplorerConfig vestibule_config = tour_config;
  vestibule_config.footprint_clearance = .30;
  vestibule_config.required_frontier_observations = 1;
  navigation2d::FrontierExplorer vestibule_explorer(vestibule_config);
  vestibule_explorer.UpdateMap(vestibule);
  const auto vestibule_goals = vestibule_explorer.SelectGoals((12 + .5) * vestibule.resolution,
                                                              (20 + .5) * vestibule.resolution);
  assert(!vestibule_goals.empty());
  assert(vestibule_explorer.candidate_frontier_goals() > 0);
}
