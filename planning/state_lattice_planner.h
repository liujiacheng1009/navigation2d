#pragma once

#include <optional>

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/planning/global_planner.h"
#include "navigation2d/planning/obstacle_heuristic.h"

namespace navigation2d {

class StateLatticePlanner final : public GlobalPlanner {
 public:
  explicit StateLatticePlanner(NavigationConfig config) : config_(std::move(config)) {}
  Path Plan(const LayeredCostmap& costmap, const Pose2d& start,
            const Pose2d& goal) const override;
  GlobalPlanningDiagnostics Diagnostics() const override { return diagnostics_; }

 private:
  NavigationConfig config_;
  mutable std::optional<planning_internal::ObstacleHeuristic> obstacle_heuristic_;
  mutable GlobalPlanningDiagnostics diagnostics_;
};

}  // namespace navigation2d
