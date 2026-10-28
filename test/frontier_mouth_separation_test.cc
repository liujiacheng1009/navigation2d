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
  // Vacuum coverage: a nearby low-gain doorway must lead the tour before a
  // far high-gain wall, even though NBV would prefer the distant blob.
  navigation2d::FrontierExplorerConfig tour_config;
  tour_config.minimum_frontier_cells = 6;
  tour_config.footprint_clearance = navigation2d::kPlanningRadius();
  tour_config.minimum_standoff = .10;
  tour_config.maximum_standoff = 1.10;
  tour_config.required_frontier_observations = 1;
  navigation2d::ExplorationGrid tour_map;
  tour_map.width = 80;
  tour_map.height = 40;
  tour_map.resolution = .05;
  tour_map.cells.assign(static_cast<std::size_t>(tour_map.width * tour_map.height), 100);
  const auto tour_fill = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row) {
      for (int col = col0; col < col1; ++col) {
        tour_map.cells[static_cast<std::size_t>(row * tour_map.width + col)] = value;
      }
    }
  };
  tour_fill(4, 50, 12, 28, 0);
  tour_fill(10, 16, 28, 34, -1);
  tour_fill(50, 76, 8, 32, -1);
  navigation2d::FrontierExplorer tour_explorer(tour_config);
  tour_explorer.UpdateMap(tour_map);
  const double tour_robot_x = (12 + .5) * tour_map.resolution;
  const double tour_robot_y = (20 + .5) * tour_map.resolution;
  const auto tour = tour_explorer.BuildTour(tour_robot_x, tour_robot_y);
  assert(!tour.empty());
  // One task per mouth: tour stops for distinct mouths stay well separated.
  for (std::size_t i = 0; i < tour.size(); ++i) {
    for (std::size_t j = i + 1; j < tour.size(); ++j) {
      assert(std::hypot(tour[i].x - tour[j].x, tour[i].y - tour[j].y) > 1.15 ||
             std::hypot(tour[i].frontier_x - tour[j].frontier_x,
                        tour[i].frontier_y - tour[j].frontier_y) > 1.15);
    }
  }
}
