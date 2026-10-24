#pragma once
#include <cstddef>

#include "navigation2d/exploration/planning_grid_view.h"
namespace navigation2d::frontier_internal {
// Synchronous read-only planning reachability, with observation authorizing
// unknown-cell accounting. Coordinates are world-frame metres. The maps must
// outlive this view and retain matching dimensions/storage for each query.
class FrontierReachability {
 public:
  FrontierReachability(const ExplorationGrid& planning, const ExplorationGrid& observation,
                       double footprint_clearance)
      : map_(planning),
        observation_(observation),
        geometry_(planning, observation, footprint_clearance) {}
  FrontierReachability(const ExplorationGrid&&, const ExplorationGrid&, double) = delete;
  FrontierReachability(const ExplorationGrid&, const ExplorationGrid&&, double) = delete;
  bool SameRegion(double anchor_x, double anchor_y, double target_x, double target_y,
                  double max_distance_m) const;
  std::size_t UnknownCells(double robot_x, double robot_y) const;

 private:
  const ExplorationGrid& map_;
  const ExplorationGrid& observation_;
  PlanningGridView geometry_;
};
}  // namespace navigation2d::frontier_internal
