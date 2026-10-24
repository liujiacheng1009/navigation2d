#pragma once
#include <Eigen/Core>

#include "navigation2d/control/pursuit_parameters.h"
#include "navigation2d/costmap/layered_costmap.h"
#include "navigation2d/geometry/path_2d.h"
#include "navigation2d/geometry/twist_2d.h"
namespace navigation2d::pursuit_internal {
// Pure nominal pursuit command. Path/carrot/pose share a metric frame;
// clearance regulation does not replace collision checking or acceleration limits.
Twist2d TrackingTarget(const Path&, const Pose2d&, const Eigen::Vector2d& carrot,
                       const LayeredCostmap&, const RegulationParameters&);
}  // namespace navigation2d::pursuit_internal
