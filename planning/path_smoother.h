#pragma once

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/planning/global_planner.h"

namespace navigation2d::planning_internal {

class ConstrainedPathSmoother {
 public:
  explicit ConstrainedPathSmoother(const NavigationConfig& config) : config_(config) {}
  Path Smooth(const Path& path, const LayeredCostmap& costmap, double robot_radius) const;

 private:
  NavigationConfig config_;
};

}  // namespace navigation2d::planning_internal
