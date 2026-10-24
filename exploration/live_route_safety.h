#pragma once
#include <utility>
#include <vector>

#include "navigation2d/exploration/frontier_types.h"
#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d {
// True when a world-frame polyline puts the body on a live occupied cell.
// Unknown is not lethal. SLAM occupied is never carved open for passage.
bool PathCutsLiveOccupied(const ExplorationGrid& live,
                          const std::vector<std::pair<double, double>>& path_xy, double robot_x,
                          double robot_y, double body_radius = kPlanningRadius(),
                          double under_robot_m = kPlanningRadius());

// After repeated live-map path cuts (>=2), leave the tip pocket instead of
// replan thrash. A single low-clearance graze near a passable door must not
// abandon the room.
bool PreferTipAbandonAfterPathCut(int mapped_obstacle_rejects, double path_min_clearance_m);

}  // namespace navigation2d
