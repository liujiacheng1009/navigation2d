#pragma once

#include <cstdint>
#include <vector>

#include "navigation2d/costmap/layered_costmap.h"

namespace navigation2d::planning_internal {

// Reverse 2D cost-aware Dijkstra field. The owning planner retains it across
// replans and rebuilds only when map digest, goal cell, or clearance changes.
class ObstacleHeuristic {
 public:
  ObstacleHeuristic(const LayeredCostmap& costmap, int goal_x, int goal_y,
                    double clearance, double cost_penalty);
  bool Matches(const LayeredCostmap& costmap, int goal_x, int goal_y,
               double clearance, double cost_penalty) const;
  double cost(int x, int y) const;

 private:
  int width_ = 0;
  int height_ = 0;
  int goal_x_ = 0;
  int goal_y_ = 0;
  double clearance_ = 0.;
  double cost_penalty_ = 0.;
  std::uint64_t digest_ = 0;
  std::vector<double> costs_;
};

}  // namespace navigation2d::planning_internal
