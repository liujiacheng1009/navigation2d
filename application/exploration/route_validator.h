#pragma once
#include <optional>
#include <utility>
#include <vector>

#include "navigation2d/application/exploration/planning_snapshot.h"
namespace navigation2d::exploration {
// live and path are call-local working copies, supplied in map coordinates.
// Planning acceptance uses the planning circle; execution uses the chassis.
bool RouteCutsMappedObstacle(ExplorationGrid live, std::vector<std::pair<double, double>> path_xy,
                             const PlanningSnapshot& snapshot, double robot_x, double robot_y,
                             double robot_yaw, const std::optional<ScanEvidenceView>& scan,
                             bool execution_tick);
}  // namespace navigation2d::exploration
