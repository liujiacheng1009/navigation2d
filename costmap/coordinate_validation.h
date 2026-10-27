#pragma once
#include <cmath>
#include <limits>

#include "navigation2d/costmap/grid_2d.h"
#include "navigation2d/geometry/pose_2d.h"

namespace navigation2d::costmap_internal {
// Index representability, not footprint admission: ordinary out-of-map poses
// still reach the planner's original occupied/nearest-obstacle diagnostics.
inline bool IndexablePose(const Grid2d& grid, const Pose2d& pose) {
  if (!std::isfinite(grid.resolution()) || grid.resolution() <= 0. || !pose.matrix().allFinite()) {
    return false;
  }
  const double limit = static_cast<double>(std::numeric_limits<int>::max()) - 1.;
  return std::abs(X(pose) / grid.resolution()) < limit &&
         std::abs(Y(pose) / grid.resolution()) < limit;
}
}  // namespace navigation2d::costmap_internal
