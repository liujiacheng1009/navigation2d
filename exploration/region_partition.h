#pragma once

#include <vector>

namespace navigation2d {
struct ExplorationGrid;
namespace exploration_internal {
// Raster components only: no stable region identity, history or DFS state.
// The caller supplies a valid nonempty observed grid and clearance in metres.
// Labels use zero for non-free cells and deterministic IDs [1, count].
struct RegionPartition {
  std::vector<int> components;
  int count = 0;
};
RegionPartition PartitionRegions(const ExplorationGrid& grid, double clearance);
}  // namespace exploration_internal
}  // namespace navigation2d
