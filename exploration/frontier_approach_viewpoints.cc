#include "navigation2d/exploration/frontier_viewpoint_search.h"

#include <algorithm>
#include <cmath>

namespace navigation2d::frontier_internal {
std::optional<ExplorationGoal> SearchNarrowViewpoint(const FrontierViewpointProblem& problem) {
  const auto& map = problem.map;
  const auto& config = problem.config;
  const auto& safe = problem.safe;
  const auto& travel_cells = problem.travel_cells;
  const auto& [mean_col, mean_row, representative, frontier_x, frontier_y] = problem.anchor;
  const auto geometry = problem.Geometry();
  const ObservationGridView raw(problem.observation);
  const int search = static_cast<int>(std::ceil(config.maximum_standoff / map.resolution));
  std::optional<ExplorationGoal> best;
  for (int row = std::max(0, static_cast<int>(mean_row) - search * 2);
       row <= std::min(map.height - 1, static_cast<int>(mean_row) + search * 2); ++row) {
    for (int col = std::max(0, static_cast<int>(mean_col) - search * 2);
         col <= std::min(map.width - 1, static_cast<int>(mean_col) + search * 2); ++col) {
      const int viewpoint_index = row * map.width + col;
      if (!safe[viewpoint_index] || travel_cells[viewpoint_index] < 0 || !raw.Free(col, row) ||
          !NarrowViewpoint(problem, col, row)) {
        continue;
      }
      const auto [x, y] = geometry.CellCenter(col, row);
      if (problem.Blacklisted(x, y)) {
        continue;
      }
      const double standoff = std::hypot(x - frontier_x, y - frontier_y);
      if (standoff < kCloseStandoff() || standoff > 1.5) {
        continue;
      }
      const auto candidate =
          MakeComponentCandidate(problem, col, row, frontier_x, frontier_y, standoff, 1.);
      if (candidate && (!best || candidate->score > best->score)) {
        best = candidate;
      }
    }
  }
  return best;
}

std::optional<ExplorationGoal> SearchOpenApproach(const FrontierViewpointProblem& problem) {
  const auto& map = problem.map;
  const auto& safe = problem.safe;
  const auto& travel_cells = problem.travel_cells;
  const auto& cluster = problem.cluster;
  const auto geometry = problem.Geometry();
  const ObservationGridView raw(problem.observation);
  std::optional<ExplorationGoal> best;
  constexpr double kApproachRadius = 2.5;
  const int approach_search = static_cast<int>(std::ceil(kApproachRadius / map.resolution));
  for (const int frontier_cell : cluster) {
    const int target_row = frontier_cell / map.width;
    const int target_col = frontier_cell % map.width;
    for (int row = std::max(0, target_row - approach_search);
         row <= std::min(map.height - 1, target_row + approach_search); ++row) {
      for (int col = std::max(0, target_col - approach_search);
           col <= std::min(map.width - 1, target_col + approach_search); ++col) {
        const int viewpoint_index = row * map.width + col;
        if (!safe[viewpoint_index] || travel_cells[viewpoint_index] < 0 || !raw.Free(col, row)) {
          continue;
        }
        const auto [x, y] = geometry.CellCenter(col, row);
        if (problem.Blacklisted(x, y)) {
          continue;
        }
        const auto [target_x, target_y] = geometry.CellCenter(target_col, target_row);
        const double standoff = std::hypot(x - target_x, y - target_y);
        if (standoff < kCloseStandoff() || standoff > kApproachRadius) {
          continue;
        }
        const auto candidate =
            MakeComponentCandidate(problem, col, row, target_x, target_y, standoff, .8);
        if (candidate && (!best || candidate->score > best->score)) {
          best = candidate;
        }
      }
    }
  }
  return best;
}
}  // namespace navigation2d::frontier_internal
