#pragma once
#include <concepts>
#include <utility>

#include "navigation2d/planning/global_planner.h"
#include "navigation2d/planning/smoother_parameters.h"

namespace navigation2d::planning_internal {

class ConstrainedPathSmoother {
 public:
  explicit ConstrainedPathSmoother(const NavigationConfig& config);
  template <std::same_as<SmootherParameters> Parameters>
  explicit ConstrainedPathSmoother(Parameters config) : config_(config) {}
  Path Smooth(const Path& path, const LayeredCostmap& costmap, double robot_radius) const;

 private:
  SmootherParameters config_;
};

}  // namespace navigation2d::planning_internal
