#include "navigation2d/exploration/leftover_viewpoints.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/exploration/doorway_geometry.h"
#include "navigation2d/exploration/frontier_travel_graph.h"
#include "navigation2d/exploration/planning_grid_view.h"
namespace navigation2d::frontier_internal {
LeftoverSelection SelectLeftoverViewpoints(const LeftoverSelectionProblem& problem, double robot_x,
                                           double robot_y, int max_count) {
  const auto& map = problem.planning;
  const PlanningGridView geometry(map, problem.observation, problem.footprint_clearance);
  const ObservationGridView observed(problem.observation);
  LeftoverSelection result;
  auto& leftovers = result.goals;
  if (max_count <= 0 || map.width <= 2 || map.height <= 2 || map.resolution <= 0. ||
      map.cells.size() != static_cast<std::size_t>(map.width * map.height)) {
    return result;
  }
  std::vector<unsigned char> frontier(map.cells.size(), 0);
  const auto graph = BuildSafeFrontierTravelGraph(map, problem.observation,
                                                  problem.footprint_clearance, robot_x, robot_y);
  if (!graph) {
    return result;
  }
  const auto& travel_cells = graph->travel_cells;
  for (int row = 0; row < map.height; ++row) {
    for (int col = 0; col < map.width; ++col) {
      const int index = row * map.width + col;
      if (observed.Frontier(col, row)) {
        frontier[index] = 1;
      }
    }
  }
  result.raw_frontier_cells = observed.CountFrontiers();
  if (*result.raw_frontier_cells == 0) {
    return result;
  }
  // Leftover free/unknown specks are often smaller than
  // minimum_frontier_cells, so the exploring tour drops them.  The pocket
  // they belong to is usually still reachable through ordinary open space.
  // Place a normal viewpoint on that open graph; the planner takes the wide
  // route, it does not squeeze a 1.2 m end gap.
  const int leftover_search = static_cast<int>(std::ceil(kLeftoverRadius / map.resolution));
  struct RankedLeftover {
    ExplorationGoal goal;
    bool room_mouth = false;
  };
  std::vector<RankedLeftover> ranked;
  // Completing must still see mouths that narrow failure cools hid from the
  // exploring tour. Wide pocket cools (>=1.5 m, tip-leave / thrash) must
  // suppress leftovers too — otherwise the robot re-enters a wall tip with a
  // frontier_cells=0 retreat ping-pong and never escapes.
  const std::size_t known_now = observed.KnownCells();
  for (int index = 0; index < map.width * map.height; ++index) {
    if (!frontier[index]) {
      continue;
    }
    const int target_row = index / map.width, target_col = index % map.width;
    const auto [frontier_x, frontier_y] = geometry.CellCenter(target_col, target_row);
    if (LeftoverSuppressed(problem, frontier_x, frontier_y, known_now)) {
      continue;
    }
    // Skip tips with no reachable approach cell — wall-external unknown.
    bool approachable = travel_cells[static_cast<std::size_t>(index)] >= 0;
    if (!approachable) {
      for (int dy = -1; dy <= 1 && !approachable; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int nc = target_col + dx, nr = target_row + dy;
          if (nc < 0 || nr < 0 || nc >= map.width || nr >= map.height) {
            continue;
          }
          if (travel_cells[static_cast<std::size_t>(nr * map.width + nc)] >= 0) {
            approachable = true;
            break;
          }
        }
      }
    }
    if (!approachable) {
      continue;
    }
    const bool room_mouth = CorridorMouthAt(map, frontier_x, frontier_y);
    const auto best_for_tip =
        BestLeftoverApproach(problem, *graph, known_now, index, leftover_search);
    if (!best_for_tip) {
      continue;
    }
    ranked.push_back({*best_for_tip, room_mouth});
  }
  std::sort(ranked.begin(), ranked.end(), [](const RankedLeftover& a, const RankedLeftover& b) {
    if (a.room_mouth != b.room_mouth) {
      return a.room_mouth && !b.room_mouth;
    }
    return a.goal.score > b.goal.score;
  });
  for (const auto& entry : ranked) {
    const bool duplicate =
        std::any_of(leftovers.begin(), leftovers.end(), [&](const ExplorationGoal& prior) {
          return std::hypot(prior.frontier_x - entry.goal.frontier_x,
                            prior.frontier_y - entry.goal.frontier_y) < .80 ||
                 std::hypot(prior.x - entry.goal.x, prior.y - entry.goal.y) < .80;
        });
    if (duplicate) {
      continue;
    }
    leftovers.push_back(entry.goal);
    if (static_cast<int>(leftovers.size()) >= max_count) {
      break;
    }
  }
  return result;
}

}  // namespace navigation2d::frontier_internal
