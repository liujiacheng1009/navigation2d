#include <cassert>

#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/test/frontier_test_scenes.h"

int main() {
  auto input = navigation2d::test::MatureLeftoverMap();
  const auto original = input;
  navigation2d::FrontierExplorer explorer;
  explorer.UpdateMap(input);
  explorer.SelectGoals(5., 5.);
  const auto& owned = explorer.observation_map();
  assert(owned.width == original.width && owned.height == original.height);
  assert(owned.resolution == original.resolution && owned.cells == original.cells);
  input.cells.assign(input.cells.size(), 100);
  assert(owned.cells == original.cells);
  explorer.UpdateMap(input);
  assert(explorer.observation_map().cells == input.cells);
}
