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
  auto door_config = navigation2d::test::DoorFrontierConfig();
  // Opening a partially observed passage is a planning-only transformation.
  // The mapper's unknown boundary must remain visible to completion logic and
  // the reported known area must not grow just because a narrow gap was
  // opened in the private navigation graph.
  navigation2d::ExplorationGrid raw_opening;
  raw_opening.width = raw_opening.height = 40;
  raw_opening.resolution = .05;
  raw_opening.cells.assign(static_cast<std::size_t>(raw_opening.width * raw_opening.height), 100);
  for (int row = 10; row < 30; ++row) {
    for (int col = 4; col < 22; ++col) {
      raw_opening.cells[row * raw_opening.width + col] = 0;
    }
  }
  for (int row = 16; row < 26; ++row) {
    for (int col = 22; col < 30; ++col) {
      raw_opening.cells[row * raw_opening.width + col] = -1;
    }
  }
  const auto raw_known =
      static_cast<std::size_t>(std::count_if(raw_opening.cells.begin(), raw_opening.cells.end(),
                                             [](std::int8_t value) { return value >= 0; }));
  navigation2d::FrontierExplorer raw_explorer(door_config);
  raw_explorer.UpdateMap(raw_opening);
  assert(raw_explorer.KnownCells() == raw_known);
  assert(raw_explorer.unresolved_frontier_cells() > 0);
  assert(!raw_explorer.SelectGoals(.5, .95).empty());
}
