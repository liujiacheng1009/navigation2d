#pragma once
#include "navigation2d/planning/global_planner.h"

namespace navigation2d::planning_internal {
// Occupied-only clearance and curvature of the prepared route. Unknown is
// excluded from the distance field, preserving the frontier planning contract.
void AddPathMetrics(const Path& path, const LayeredCostmap& costmap, double robot_radius,
                    GlobalPlanningDiagnostics* diagnostics);
}  // namespace navigation2d::planning_internal
