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
  auto leftover_config = navigation2d::test::LeftoverFrontierConfig();
  auto leftover = navigation2d::test::MatureLeftoverMap();
  // Mature open room with a >=minimum_frontier_cells residual strip against a
  // furniture wall: SelectGoals must not place a tour GOAL in already-explored
  // free space (the red-box junction thrash). Completing leftovers may still
  // see the fragment.
  navigation2d::ExplorationGrid residual = leftover;
  for (int row = 40; row < 49; ++row) {
    residual.cells[static_cast<std::size_t>(row * residual.width + 20)] = -1;
  }
  for (int row = 40; row < 49; ++row) {
    residual.cells[static_cast<std::size_t>(row * residual.width + 19)] = 100;
  }
  navigation2d::FrontierExplorer residual_explorer(leftover_config);
  residual_explorer.UpdateMap(residual);
  assert(residual_explorer.KnownCells() >= 10000);
  assert(residual_explorer.raw_frontier_cells() >= 6);
  const auto residual_tour = residual_explorer.SelectGoals(5., 5.);
  assert(residual_tour.empty());
  assert(residual_explorer.SelectLeftoverApproach(5., 5.).has_value());
}
