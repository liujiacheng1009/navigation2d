#pragma once
#include <span>
#include <vector>

#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d {
// Debug overlay for SelectGoals: approachable frontier cells, per-cluster
// viewpoints, and final mouth-stable goals. World frame = map frame.
struct FrontierDebugSnapshot {
  struct Cell {
    double x = 0.;
    double y = 0.;
    int cluster_id = -1;
  };
  struct Goal {
    double x = 0.;
    double y = 0.;
    double frontier_x = 0.;
    double frontier_y = 0.;
    int cluster_id = -1;
    double score = 0.;
    double information_gain = 0.;
    int frontier_cells = 0;
  };
  std::vector<Cell> cells;
  std::vector<Goal> cluster_goals;
  std::vector<Goal> goals;
};

namespace frontier_internal {
// Only the snapshot is mutated. Live refresh retains previous cluster/stable
// goals; stable-goal association uses strict nearest distance and original ties.
void RefreshFrontierDebugCells(const ExplorationGrid& observation, FrontierDebugSnapshot& snapshot);
void AppendStableFrontierDebug(FrontierDebugSnapshot& snapshot,
                               std::span<const ExplorationGoal> stable);
}  // namespace frontier_internal
}  // namespace navigation2d
