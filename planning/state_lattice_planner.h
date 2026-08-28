#pragma once

#include <optional>
#include <memory>

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/planning/global_planner.h"
#include "navigation2d/planning/obstacle_heuristic.h"
#include "navigation2d/planning/collision_checker.h"
#include "navigation2d/planning/incremental_search.h"
#include "navigation2d/planning/motion_primitives.h"

namespace navigation2d {

class StateLatticePlanner final : public GlobalPlanner {
 public:
  explicit StateLatticePlanner(NavigationConfig config) : config_(std::move(config)) {}
  ~StateLatticePlanner() override = default;
  Path Plan(const LayeredCostmap& costmap, const Pose2d& start,
            const Pose2d& goal) const override;
  GlobalPlanningDiagnostics Diagnostics() const override { return diagnostics_; }

 private:
  NavigationConfig config_;
  mutable std::optional<planning_internal::ObstacleHeuristic> obstacle_heuristic_;
  mutable GlobalPlanningDiagnostics diagnostics_;
  mutable const LayeredCostmap* active_costmap_ = nullptr;
  mutable std::unique_ptr<planning_internal::DistanceField> distance_field_;
  mutable std::optional<planning_internal::DifferentialDrivePrimitiveSet> primitives_;
  mutable std::unique_ptr<planning_internal::DStarLite> incremental_search_;
  mutable int lattice_width_ = 0;
  mutable int lattice_height_ = 0;
  mutable int incremental_goal_ = -1;
  mutable std::uint64_t map_revision_ = 0;

  int Encode(int x, int y, int yaw_bin) const;
  void Decode(int state, int* x, int* y, int* yaw_bin) const;
  std::optional<double> PrimitiveCost(int x, int y,
      const planning_internal::MotionPrimitive& primitive) const;
  void VisitSuccessors(int state, const planning_internal::SuccessorVisitor& visit) const;
  void VisitPredecessors(int state, const planning_internal::SuccessorVisitor& visit) const;
  double PairHeuristic(int first, int second) const;
};

}  // namespace navigation2d
