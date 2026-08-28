#pragma once

#include <vector>

#include <Eigen/Core>

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/costmap/layered_costmap.h"
#include "navigation2d/planning/global_planner.h"

namespace navigation2d::control_internal {

struct Halfspace2d {
  Eigen::Vector2d normal;
  double bound = 0.;
};

using ConvexCorridor = std::vector<std::vector<Halfspace2d>>;

// ROS-free adapter around DecompUtil's EllipsoidDecomp2D. Bounds are returned
// as n.dot(robot_center) <= bound after footprint support-function shrinking.
ConvexCorridor BuildSafeCorridor(const Path& path, const LayeredCostmap& costmap,
                                 const NavigationConfig& config);

}  // namespace navigation2d::control_internal
