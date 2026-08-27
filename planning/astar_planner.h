#pragma once

#include "navigation2/planning/global_planner.h"

namespace navigation2d {

class AStarPlanner final : public GlobalPlanner {
 public:
  explicit AStarPlanner(double clearance) : clearance_(clearance) {}
  Path Plan(const LayeredCostmap&, const Pose2d&, const Pose2d&) const override;

 private:
  double clearance_;
};

}  // namespace navigation2d
