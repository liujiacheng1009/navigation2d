#pragma once
#include <optional>
#include <span>

#include "navigation2d/exploration/exploration_config.h"
#include "navigation2d/exploration/frontier_components.h"
#include "navigation2d/exploration/frontier_visit_memory.h"
#include "navigation2d/exploration/planning_grid_view.h"
namespace navigation2d::frontier_internal {
// Read-only inputs to a single component search. Every borrow ends when the
// selection call returns. Masks/travel cover the complete planning grid and
// cluster cells/anchor refer to that grid. No mutable mission state is retained.
struct FrontierViewpointProblem {
  const ExplorationGrid& map;
  const ExplorationGrid& observation;
  const FrontierExplorerConfig& config;
  const FrontierVisitMemory& visits;
  std::span<const unsigned char> safe;
  std::span<const int> travel_cells;
  std::span<const int> cluster;
  ClusterAnchor anchor;
  bool Blacklisted(double x, double y) const {
    return visits.Contains(x, y, false, ObservationGridView(observation).KnownCells());
  }
  PlanningGridView Geometry() const { return {map, observation, config.footprint_clearance}; }
};
[[nodiscard]] bool NarrowViewpoint(const FrontierViewpointProblem&, int col, int row);
[[nodiscard]] std::optional<ExplorationGoal> MakeComponentCandidate(
    const FrontierViewpointProblem&, int col, int row, double target_x, double target_y,
    double standoff, double visibility_weight);
// Fallback priority is strict: representative LOS, any member LOS, narrow
// corridor approach, then bounded open approach. A later pass cannot replace
// an earlier pass's candidate merely because its score is larger.
[[nodiscard]] std::optional<ExplorationGoal> SelectComponentViewpoint(
    const FrontierViewpointProblem&);
[[nodiscard]] std::optional<ExplorationGoal> SearchVisibleViewpoint(const FrontierViewpointProblem&,
                                                                    bool any_member);
[[nodiscard]] std::optional<ExplorationGoal> SearchNarrowViewpoint(const FrontierViewpointProblem&);
[[nodiscard]] std::optional<ExplorationGoal> SearchOpenApproach(const FrontierViewpointProblem&);
}  // namespace navigation2d::frontier_internal
