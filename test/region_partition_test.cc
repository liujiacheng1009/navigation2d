#include "navigation2d/exploration/region_partition.h"

#include <cassert>
#include <vector>

#include "navigation2d/exploration/frontier_types.h"

int main() {
  navigation2d::ExplorationGrid grid;
  grid.width = 33;
  grid.height = 17;
  grid.resolution = .1;
  grid.cells.assign(grid.width * grid.height, 100);
  auto set = [&](int x, int y, int value) { grid.cells[y * grid.width + x] = value; };
  for (int y = 2; y <= 14; ++y) {
    for (int x = 2; x <= 14; ++x) { set(x, y, 0); }
    for (int x = 18; x <= 30; ++x) { set(x, y, 0); }
  }
  // A one-cell doorway is connected observed space but separates room cores.
  for (int x = 15; x <= 17; ++x) { set(x, 8, 0); }
  // A narrow disconnected island must survive even when broad cores exist.
  set(16, 1, 0);
  auto partition = navigation2d::exploration_internal::PartitionRegions(grid, .15);
  auto label = [&](int x, int y) { return partition.components[y * grid.width + x]; };
  assert(partition.count == 3);
  assert(label(5, 8) != 0 && label(25, 8) != 0);
  assert(label(5, 8) != label(25, 8));
  assert(label(16, 1) != 0 && label(16, 1) != label(5, 8));
  for (std::size_t i = 0; i < grid.cells.size(); ++i) {
    assert((partition.components[i] != 0) == (grid.cells[i] == 0));
  }
  // With no eroded cores, a connected corridor remains one region.
  grid.cells.assign(grid.cells.size(), -1);
  for (int x = 1; x < 32; ++x) { set(x, 8, 0); }
  partition = navigation2d::exploration_internal::PartitionRegions(grid, .15);
  assert(partition.count == 1);
  for (int x = 1; x < 32; ++x) { assert(label(x, 8) == 1); }
  // Fully occupied/unknown maps have no manufactured free-space identity.
  grid.cells.assign(grid.cells.size(), 100);
  partition = navigation2d::exploration_internal::PartitionRegions(grid, .15);
  assert(partition.count == 0);
  for (int value : partition.components) { assert(value == 0); }
}
