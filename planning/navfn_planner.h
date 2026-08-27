#pragma once

#include "navigation2/types.h"
#include "navigation2/costmap/layered_costmap.h"

namespace navigation2d {

// ROS-free adapter for NavFn's grid-based shortest-path contract.
class NavFnPlanner {
 public:
  NavFnPlanner(double clearance, std::string algorithm)
      : clearance_(clearance), algorithm_(std::move(algorithm)) {}
  Path Plan(const LayeredCostmap& costmap, const Pose2d& start, const Pose2d& goal) const;

 private:
  double clearance_;
  std::string algorithm_;
};

}  // namespace navigation2d
#include <string>
