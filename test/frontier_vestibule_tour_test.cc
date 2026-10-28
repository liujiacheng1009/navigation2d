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
  // Vestibule: both free runs are door-width (~0.50 m). SafeViewpoint must
  // still accept the center with planning radius (.22); the old XOR treated
  // both-narrow as open space and demanded footprint_clearance (.30).
  {
    navigation2d::FrontierExplorerConfig vestibule_config;
    vestibule_config.footprint_clearance = .30;
    vestibule_config.minimum_frontier_cells = 6;
    vestibule_config.minimum_standoff = .08;
    vestibule_config.maximum_standoff = 1.10;
    vestibule_config.required_frontier_observations = 1;
    navigation2d::ExplorationGrid vestibule;
    vestibule.width = 40;
    vestibule.height = 40;
    vestibule.resolution = .05;
    vestibule.cells.assign(static_cast<std::size_t>(vestibule.width * vestibule.height), 100);
    for (int row = 10; row < 20; ++row) {
      for (int col = 10; col < 30; ++col) {
        vestibule.cells[static_cast<std::size_t>(row * vestibule.width + col)] = 0;
      }
    }
    // 0.50 m × 0.50 m kitchen vestibule past a hall, then unknown room.
    for (int row = 20; row < 30; ++row) {
      for (int col = 15; col < 25; ++col) {
        vestibule.cells[static_cast<std::size_t>(row * vestibule.width + col)] = 0;
      }
    }
    for (int row = 30; row < 38; ++row) {
      for (int col = 15; col < 25; ++col) {
        vestibule.cells[static_cast<std::size_t>(row * vestibule.width + col)] = -1;
      }
    }
    navigation2d::FrontierExplorer vestibule_explorer(vestibule_config);
    vestibule_explorer.UpdateMap(vestibule);
    const double robot_x = (20 + .5) * vestibule.resolution;
    const double robot_y = (14 + .5) * vestibule.resolution;
    const auto goals = vestibule_explorer.SelectGoals(robot_x, robot_y);
    assert(!goals.empty());
    const auto tour = vestibule_explorer.BuildTour(robot_x, robot_y);
    assert(!tour.empty());
  }
}
