#pragma once

#include <functional>

#include "navigation2/planning/global_planner.h"

namespace navigation2d::planning_internal {

using Heuristic = std::function<double(int, int, int, int)>;
using ParentRelaxation = std::function<bool(const LayeredCostmap&, int, int, int,
                                            double, double, int*, double*)>;

Path GridSearch(const LayeredCostmap& costmap, const Pose2d& start,
                const Pose2d& goal, double clearance, const Heuristic& heuristic,
                const ParentRelaxation& relax_parent = {});
bool HasLineOfSight(const LayeredCostmap& costmap, int from, int to,
                    double clearance);
Path DensifyPath(const Path& path, double spacing);

}  // namespace navigation2d::planning_internal
