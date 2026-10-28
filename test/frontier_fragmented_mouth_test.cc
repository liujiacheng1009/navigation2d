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
  // Fragmented frontier cells along one alcove mouth must collapse to a single
  // tour stop — not a pile of micro-cluster T-labels.
  navigation2d::ExplorationGrid alcove;
  alcove.width = 60;
  alcove.height = 40;
  alcove.resolution = .05;
  alcove.cells.assign(static_cast<std::size_t>(alcove.width * alcove.height), 100);
  const auto alcove_fill = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row) {
      for (int col = col0; col < col1; ++col) {
        alcove.cells[static_cast<std::size_t>(row * alcove.width + col)] = value;
      }
    }
  };
  alcove_fill(4, 40, 8, 32, 0);
  // Jagged unknown pocket (~1 m wide) that splits into several frontier clusters.
  alcove_fill(18, 28, 32, 38, -1);
  alcove.cells[static_cast<std::size_t>(31 * alcove.width + 20)] = -1;
  alcove.cells[static_cast<std::size_t>(31 * alcove.width + 22)] = -1;
  alcove.cells[static_cast<std::size_t>(31 * alcove.width + 25)] = -1;
  navigation2d::FrontierExplorer alcove_explorer(tour_config);
  alcove_explorer.UpdateMap(alcove);
  const auto alcove_tour =
      alcove_explorer.BuildTour((20 + .5) * alcove.resolution, (18 + .5) * alcove.resolution);
  assert(!alcove_tour.empty());
  assert(alcove_tour.size() == 1);
}
