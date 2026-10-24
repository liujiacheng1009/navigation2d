#pragma once
#include <optional>
#include <utility>
#include <vector>

#include "navigation2d/exploration/planning_grid_view.h"
namespace navigation2d::frontier_internal {
// Synchronous immutable planning/observed views. Maps must outlive each query;
// world-frame metres and the original four-neighbour/tie order are retained.
class EntryProbeGrid {
 public:
  EntryProbeGrid(const ExplorationGrid& map, const ExplorationGrid& observation, double clearance)
      : map_(map), observation_(observation), geometry_(map, observation, clearance) {}
  EntryProbeGrid(const ExplorationGrid&&, const ExplorationGrid&, double) = delete;
  EntryProbeGrid(const ExplorationGrid&, const ExplorationGrid&&, double) = delete;
  bool Allows(int col, int row) const;
  std::vector<int> Distances(double robot_x, double robot_y) const;

 private:
  const ExplorationGrid& map_;
  const ExplorationGrid& observation_;
  PlanningGridView geometry_;
};
std::optional<std::pair<double, double>> EntryProbeDirection(const ExplorationGrid&,
                                                             const ExplorationGrid&,
                                                             const ExplorationGoal&, double robot_x,
                                                             double robot_y);
double MouthGapWidthAt(const ExplorationGrid&, double frontier_x, double frontier_y);
std::optional<ExplorationGoal> SelectEntryProbeAt(const ExplorationGrid&, const ExplorationGrid&,
                                                  double footprint_clearance,
                                                  const ExplorationGoal&, double robot_x,
                                                  double robot_y, double minimum_depth_m,
                                                  double maximum_depth_m);
}  // namespace navigation2d::frontier_internal
