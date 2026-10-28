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
  auto config = navigation2d::test::BasicFrontierConfig();
  {
    // A 0.72 m kitchen fully seen through its door from the hallway has no
    // frontier, yet the robot never drove in. It must still be a visit goal.
    navigation2d::FrontierExplorer visits(config);
    navigation2d::ExplorationGrid flat;
    flat.width = 140;
    flat.height = 130;
    flat.resolution = .03;
    flat.cells.assign(static_cast<std::size_t>(flat.width) * flat.height, 100);
    for (int row = 5; row < 39; ++row) {
      for (int col = 5; col < 135; ++col) {
        flat.cells[row * flat.width + col] = 0;
      }
    }
    for (int row = 39; row < 126; ++row) {
      for (int col = 60; col < 84; ++col) {
        flat.cells[row * flat.width + col] = 0;
      }
    }
    visits.UpdateMap(flat);
    const double pocket = navigation2d::kRoomPocketDepthM() * navigation2d::kRoomPocketDepthM();
    const std::vector<std::pair<double, double>> kitchen_door{{2.16, 1.35}};
    std::vector<std::pair<double, double>> trail;
    // Hallway driven only up to the kitchen door: its far end is mapped and
    // far from the trail, but no doorway was tried there, so it is no goal.
    for (double x = .3; x < 2.2; x += .05) {
      trail.emplace_back(x, .66);
    }
    assert(visits
               .UnvisitedSpaceGoals(trail, navigation2d::kOpenRetreat(), pocket, {},
                                    navigation2d::kOpenRetreat())
               .empty());
    const auto unvisited = visits.UnvisitedSpaceGoals(trail, navigation2d::kOpenRetreat(), pocket,
                                                      kitchen_door, navigation2d::kOpenRetreat());
    assert(unvisited.size() == 1);
    assert(unvisited.front().y > 1.2);
    assert(unvisited.front().x > 1.8 && unvisited.front().x < 2.52);
    for (double y = .66; y < 3.6; y += .05) {
      trail.emplace_back(2.16, y);
    }
    assert(visits
               .UnvisitedSpaceGoals(trail, navigation2d::kOpenRetreat(), pocket, kitchen_door,
                                    navigation2d::kOpenRetreat())
               .empty());
  }
}
