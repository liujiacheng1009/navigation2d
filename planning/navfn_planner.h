#pragma once

#include "navigation2/types.h"
#include "navigation2/costmap/layered_costmap.h"

namespace navigation2d {

// ROS-free adapter for NavFn's grid-based shortest-path contract.
class NavFnPlanner {
 public:
  explicit NavFnPlanner(double clearance) : clearance_(clearance) {}
  Path Plan(const LayeredCostmap& costmap, const Pose2d& start, const Pose2d& goal) const;

 private:
  double clearance_;
};

}  // namespace navigation2d
