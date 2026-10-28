#pragma once

#include <optional>
#include <vector>

#include "navigation2d/exploration/exploration_config.h"
#include "navigation2d/exploration/exploration_policy.h"
#include "navigation2d/exploration/frontier_debug_snapshot.h"
#include "navigation2d/exploration/frontier_types.h"
#include "navigation2d/exploration/frontier_visit_memory.h"

namespace navigation2d::frontier_internal {
struct FrontierCandidateProblem {
  const ExplorationGrid& planning;
  const ExplorationGrid& observation;
  const FrontierExplorerConfig& config;
  const FrontierVisitMemory& visits;
  const TipLedger& tips;
};
struct FrontierCandidateResult {
  std::vector<ExplorationGoal> goals;
  FrontierDebugSnapshot debug;
  std::size_t raw_frontier_clusters = 0;
  // No graph means no new reachability evidence, rather than zero unknown work.
  std::optional<std::size_t> reachable_frontier_cells;
};
// Borrowed inputs last for this synchronous call. Row-major component discovery,
// admission and score ordering are preserved. Does not update history, stable
// tracks, region commitments or mission state; the facade owns those stages.
[[nodiscard]] FrontierCandidateResult CollectFrontierCandidates(
    const FrontierCandidateProblem& problem, double robot_x, double robot_y);
}  // namespace navigation2d::frontier_internal
