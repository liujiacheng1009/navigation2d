#pragma once
#include <vector>

#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d::frontier_internal {
// Stable first-matching-mouth grouping. Retains winning task metadata and
// maximum cell/gain evidence; input order and strict comparisons are semantic.
std::vector<ExplorationGoal> MergeMouthGoals(std::vector<ExplorationGoal> goals);
// Synchronous read-only map queries. Advances only into observed free, then
// centers a corridor target using the original clearance/depth/lateral score.
std::vector<ExplorationGoal> PlaceMouthGoals(std::vector<ExplorationGoal>, const ExplorationGrid&,
                                             const ExplorationGrid&, double footprint_clearance,
                                             double robot_x, double robot_y);
}  // namespace navigation2d::frontier_internal
