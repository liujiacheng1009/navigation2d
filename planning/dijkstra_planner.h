#pragma once

#include "navigation2d/planning/global_planner.h"

namespace navigation2d {

class DijkstraPlanner final : public GlobalPlanner {
 public:
  explicit DijkstraPlanner(double clearance) : clearance_(clearance) {}
  Path Plan(const LayeredCostmap&, const Pose2d&, const Pose2d&) const override;

 private:
  double clearance_;
};

}  // namespace navigation2d
