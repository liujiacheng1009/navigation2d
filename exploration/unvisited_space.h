#pragma once
#include <utility>
#include <vector>

#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d::frontier_internal {
// Observed-free eight-neighbour distances, preserving diagonal and snap ties.
// Valid map storage is required. An optional ascent field restricts seed flood
// to cells at least as far from the driven trail as the current cell.
std::vector<double> FreeSpaceDistances(const ExplorationGrid&,
                                       const std::vector<std::pair<double, double>>&,
                                       double limit_m, int snap_cells,
                                       const std::vector<double>* ascend);
std::vector<ExplorationGoal> UnvisitedSpaceGoalsAt(
    const ExplorationGrid&, const ExplorationGrid&, double footprint_clearance,
    const std::vector<std::pair<double, double>>& trail, double visit_radius_m,
    double minimum_area_m2, const std::vector<std::pair<double, double>>& room_seeds,
    double seed_radius_m);
}  // namespace navigation2d::frontier_internal
