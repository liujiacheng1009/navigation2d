#pragma once

#include "navigation2d/costmap/layered_costmap.h"
#include "navigation2d/geometry/path_2d.h"
#include "navigation2d/planning/path_metadata.h"

namespace navigation2d::planning_internal {
// All coordinates use the same local map frame, in metres/radians.
Path DensifyRoute(const Path& sparse, double maximum_step);
// Body yaw remains body yaw; reverse and zero-translation rotations survive.
Path DensifyBodyHeadingRoute(const Path& sparse, double maximum_step);
bool PathFootprintValid(const Path& path, const LayeredCostmap& costmap, double robot_radius);
// Ordered route projection with the existing backward slack and forward window.
double PathProgress(const Path& path, const Pose2d& pose, double minimum_progress);
double InitialHeadingError(const Path& path, PathHeadingSemantics semantics, const Pose2d& pose);
double DistanceToPath(const Path& path, const Pose2d& pose);
}  // namespace navigation2d::planning_internal
