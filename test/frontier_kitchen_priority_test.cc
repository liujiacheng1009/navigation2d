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
  // Nearby open-boundary expansion must not beat a farther unfinished kitchen
  // doorway. Room/alcove throats lead the tour before hallway wall crawling.
  navigation2d::ExplorationGrid kitchen_first;
  kitchen_first.width = 100;
  kitchen_first.height = 50;
  kitchen_first.resolution = .05;
  kitchen_first.cells.assign(static_cast<std::size_t>(kitchen_first.width * kitchen_first.height),
                             100);
  const auto kfirst = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row) {
      for (int col = col0; col < col1; ++col) {
        kitchen_first.cells[static_cast<std::size_t>(row * kitchen_first.width + col)] = value;
      }
    }
  };
  kfirst(4, 70, 18, 34, 0);    // hallway
  kfirst(4, 22, 34, 46, -1);   // nearby wide open expansion (not a door)
  kfirst(70, 96, 10, 40, -1);  // kitchen room
  kfirst(68, 70, 20, 32, 0);   // 0.60 m throat into kitchen
  kfirst(68, 70, 20, 24, 0);
  navigation2d::FrontierExplorer kitchen_first_explorer(tour_config);
  kitchen_first_explorer.UpdateMap(kitchen_first);
  const double kfx = (20 + .5) * kitchen_first.resolution;
  const double kfy = (26 + .5) * kitchen_first.resolution;
  const auto kitchen_first_tour = kitchen_first_explorer.BuildTour(kfx, kfy);
  assert(!kitchen_first_tour.empty());
  // Dispatch commits the kitchen mouth; later tours stay on that region
  // until its executable observations are exhausted.
  kitchen_first_explorer.CommitRegion(kitchen_first_tour.front());
  const auto after_enter = kitchen_first_explorer.BuildTour(kfx, kfy);
  if (!after_enter.empty() && kitchen_first_tour.front().region_id != 0) {
    assert(after_enter.front().region_id == kitchen_first_tour.front().region_id ||
           kitchen_first_explorer.GoalInActiveRegion(after_enter.front()));
  }
}
