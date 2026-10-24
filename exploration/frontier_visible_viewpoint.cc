#include "navigation2d/exploration/frontier_viewpoint_search.h"

#include <algorithm>
#include <cmath>

namespace navigation2d::frontier_internal {
std::optional<ExplorationGoal> SearchVisibleViewpoint(const FrontierViewpointProblem& problem,
                                                      bool any_member) {
  const auto& map = problem.map;
  const auto& config = problem.config;
  const auto& safe = problem.safe;
  const auto& travel_cells = problem.travel_cells;
  const auto& cluster = problem.cluster;
  const auto& [mean_col, mean_row, representative, frontier_x, frontier_y] = problem.anchor;
  const auto geometry = problem.Geometry();
  const ObservationGridView raw(problem.observation);
  const int search = static_cast<int>(std::ceil(config.maximum_standoff / map.resolution));
  std::optional<ExplorationGoal> best;
  for (int row = std::max(0, static_cast<int>(mean_row) - search);
       row <= std::min(map.height - 1, static_cast<int>(mean_row) + search); ++row) {
    for (int col = std::max(0, static_cast<int>(mean_col) - search);
         col <= std::min(map.width - 1, static_cast<int>(mean_col) + search); ++col) {
      const int viewpoint_index = row * map.width + col;
      // Only planning-radius viewpoints on observed free. Carved/flooded
      // cells past a seal must not become tour goals (through-wall paths).
      if (!safe[viewpoint_index] || travel_cells[viewpoint_index] < 0 || !raw.Free(col, row)) {
        continue;
      }
      const auto [x, y] = geometry.CellCenter(col, row);
      if (problem.Blacklisted(x, y)) {
        continue;
      }
      const double standoff = std::hypot(x - frontier_x, y - frontier_y);
      if (standoff > config.maximum_standoff ||
          (standoff < config.minimum_standoff &&
           !(NarrowViewpoint(problem, col, row) && standoff >= kCloseStandoff()))) {
        continue;
      }
      const bool visible = any_member
                               ? std::any_of(cluster.begin(), cluster.end(),
                                             [&](int cell) {
                                               return geometry.HasFreeLineOfSight(
                                                   col, row, cell % map.width, cell / map.width);
                                             })
                               : geometry.HasFreeLineOfSight(col, row, representative % map.width,
                                                             representative / map.width);
      if (!visible) {
        continue;
      }
      const auto candidate = MakeComponentCandidate(problem, col, row, frontier_x, frontier_y,
                                                    standoff, any_member ? 1.5 : 2.);
      if (candidate && (!best || candidate->score > best->score)) {
        best = candidate;
      }
    }
  }
  return best;
}
}  // namespace navigation2d::frontier_internal
