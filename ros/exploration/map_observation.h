#pragma once
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navigation2d/application/exploration/pose_observation.h"
#include "navigation2d/application/exploration/route_validator.h"
#include "navigation2d/geometry/path_2d.h"
namespace navigation2d::exploration_ros {
// World-frame segment against observed occupied cells (>=50). Unknown cells
// are not walls; out-of-grid samples are blocked. A missing map is permissive,
// matching the original query; callers retain their independent input guards.
bool SegmentCutsWall(const nav_msgs::msg::OccupancyGrid* map, double x0, double y0, double x1,
                     double y1);
// Route is planning-local; origin translates it into the observed map frame.
// No pose falls back to the route start and omits scan evidence, as before.
bool ObservedRouteCutsWall(const nav_msgs::msg::OccupancyGrid* map, const Path& route,
                           double origin_x, double origin_y,
                           const exploration::PlanningSnapshot& snapshot,
                           const std::optional<exploration::PoseObservation>& robot,
                           const std::optional<exploration::ScanEvidenceView>& scan,
                           bool execution_tick);
}  // namespace navigation2d::exploration_ros
