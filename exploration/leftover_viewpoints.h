#pragma once
#include <optional>
#include <vector>

#include "navigation2d/exploration/exploration_policy.h"
#include "navigation2d/exploration/frontier_travel_graph.h"
#include "navigation2d/exploration/frontier_types.h"
#include "navigation2d/exploration/frontier_visit_memory.h"
namespace navigation2d::frontier_internal {
inline constexpr double kLeftoverRadius = 6.0;
struct LeftoverSelectionProblem {
  const ExplorationGrid& planning;
  const ExplorationGrid& observation;
  double footprint_clearance;
  const TipLedger& tips;
  const FrontierVisitMemory& visits;
};
struct LeftoverSelection {
  std::vector<ExplorationGoal> goals;
  // Invalid input/no safe start preserves the caller's previous frontier count;
  // an engaged zero is a completed scan with no raw frontier evidence.
  std::optional<std::size_t> raw_frontier_cells;
};
inline bool LeftoverSuppressed(const LeftoverSelectionProblem& problem, double x, double y,
                               std::size_t known_now) {
  if (problem.tips.Blocked(x, y) || problem.visits.Persistent(x, y)) {
    return true;
  }
  return problem.visits.WidePocket(x, y, known_now);
}
std::optional<ExplorationGoal> BestLeftoverApproach(const LeftoverSelectionProblem&,
                                                    const FrontierTravelGraph&,
                                                    std::size_t known_now, int frontier_index,
                                                    int search_radius_cells);
LeftoverSelection SelectLeftoverViewpoints(const LeftoverSelectionProblem&, double robot_x,
                                           double robot_y, int max_count);
}  // namespace navigation2d::frontier_internal
