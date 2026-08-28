#pragma once

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/planning/global_planner.h"

namespace navigation2d {

class StateLatticePlanner final : public GlobalPlanner {
 public:
  explicit StateLatticePlanner(NavigationConfig config) : config_(std::move(config)) {}
  Path Plan(const LayeredCostmap& costmap, const Pose2d& start,
            const Pose2d& goal) const override;

 private:
  NavigationConfig config_;
};

}  // namespace navigation2d
