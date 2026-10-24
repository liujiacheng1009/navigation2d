#include <algorithm>
#include <cmath>

#include "navigation2d/exploration/doorway_geometry.h"
#include "navigation2d/exploration/frontier_viewpoint_score.h"
#include "navigation2d/exploration/leftover_viewpoints.h"
#include "navigation2d/exploration/planning_grid_view.h"
namespace navigation2d::frontier_internal {
std::optional<ExplorationGoal> BestLeftoverApproach(const LeftoverSelectionProblem& problem,
                                                    const FrontierTravelGraph& graph,
                                                    std::size_t known_now, int index,
                                                    int leftover_search) {
  const auto& map = problem.planning;
  const PlanningGridView geometry(map, problem.observation, problem.footprint_clearance);
  const ObservationGridView observed(problem.observation);
  const auto& safe = graph.safe;
  const auto& travel_cells = graph.travel_cells;
  const int target_col = index % map.width, target_row = index / map.width;
  const auto [frontier_x, frontier_y] = geometry.CellCenter(target_col, target_row);
  std::optional<ExplorationGoal> best_for_tip;
  for (int row = std::max(0, target_row - leftover_search);
       row <= std::min(map.height - 1, target_row + leftover_search); ++row) {
    for (int col = std::max(0, target_col - leftover_search);
         col <= std::min(map.width - 1, target_col + leftover_search); ++col) {
      const int viewpoint_index = row * map.width + col;
      if (!safe[viewpoint_index] || travel_cells[viewpoint_index] < 0 || !observed.Free(col, row)) {
        continue;
      }
      const auto [x, y] = geometry.CellCenter(col, row);
      if (LeftoverSuppressed(problem, x, y, known_now)) {
        continue;
      }
      const double standoff = std::hypot(x - frontier_x, y - frontier_y);
      if (standoff < kCloseStandoff() || standoff > kLeftoverRadius) {
        continue;
      }
      const double clearance = OccupiedClearanceAt(map, col, row);
      if (clearance < kPlanningRadius()) {
        continue;
      }
      const double travel = travel_cells[viewpoint_index] * map.resolution;
      ExplorationGoal candidate{x,
                                y,
                                std::atan2(frontier_y - y, frontier_x - x),
                                frontier_x,
                                frontier_y,
                                1,
                                map.resolution,
                                ViewpointScore(map.resolution, travel, standoff, .8, clearance),
                                true};
      if (!best_for_tip || candidate.score > best_for_tip->score) {
        best_for_tip = candidate;
      }
    }
  }
  return best_for_tip;
}
}  // namespace navigation2d::frontier_internal
