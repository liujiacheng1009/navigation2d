#include "navigation2d/exploration/frontier_candidate_stage.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/exploration/doorway_geometry.h"
#include "navigation2d/exploration/frontier_boundary_queries.h"
#include "navigation2d/exploration/frontier_components.h"
#include "navigation2d/exploration/frontier_travel_graph.h"
#include "navigation2d/exploration/frontier_viewpoint_search.h"

namespace navigation2d::frontier_internal {
FrontierCandidateResult CollectFrontierCandidates(const FrontierCandidateProblem& problem,
                                                  double robot_x, double robot_y) {
  const auto& map = problem.planning;
  const auto& observation = problem.observation;
  FrontierCandidateResult result;
  if (map.width <= 2 || map.height <= 2 || map.resolution <= 0. ||
      map.cells.size() != static_cast<std::size_t>(map.width * map.height)) {
    return {};
  }
  frontier_internal::FrontierComponents components(map.width, map.height);
  const auto graph = frontier_internal::BuildFrontierTravelGraph(
      map, observation, problem.config.footprint_clearance, robot_x, robot_y);
  if (!graph) {
    return {};
  }
  const auto& safe = graph->safe;
  const auto& travel_cells = graph->travel_cells;
  const auto reachable_frontier_cells = components.Discover(observation, travel_cells);
  result.reachable_frontier_cells = reachable_frontier_cells;
  auto& goals = result.goals;
  while (const auto component = components.Next()) {
    const auto& cluster = *component;
    ++result.raw_frontier_clusters;
    const int cluster_id = static_cast<int>(result.raw_frontier_clusters);
    // Clusters below the minimum stay out of the exploring tour unless a cell
    // is a passable door or still has a room of unknown behind it.
    bool cluster_is_door = false;
    if (cluster.size() < static_cast<std::size_t>(problem.config.minimum_frontier_cells)) {
      for (const int cell : cluster) {
        const int cell_row = cell / map.width, cell_col = cell % map.width;
        const auto [cx, cy] = PlanningGridView(map, observation, problem.config.footprint_clearance)
                                  .CellCenter(cell_col, cell_row);
        if (CorridorMouthAt(map, cx, cy) || UnknownRoomBeyond(map, cell_col, cell_row)) {
          cluster_is_door = true;
          break;
        }
      }
      if (!cluster_is_door) {
        continue;
      }
    }
    const auto anchor = frontier_internal::LocateCluster(cluster, map);
    const double frontier_x = anchor.frontier_x, frontier_y = anchor.frontier_y;
    if (problem.visits.Contains(frontier_x, frontier_y, true,
                                ObservationGridView(observation).KnownCells())) {
      continue;
    }
    for (const int cell : cluster) {
      const auto [cx, cy] = PlanningGridView(map, observation, problem.config.footprint_clearance)
                                .CellCenter(cell % map.width, cell / map.width);
      result.debug.cells.push_back({cx, cy, cluster_id});
    }
    const auto best = frontier_internal::SelectComponentViewpoint(
        {map, observation, problem.config, problem.visits, safe, travel_cells, cluster, anchor});
    if (best) {
      ExploringGoalFacts facts;
      facts.frontier_cells = best->frontier_cells;
      facts.information_gain = best->information_gain;
      facts.cell_area = map.resolution * map.resolution;
      facts.passable_mouth = CorridorMouthAt(map, best->frontier_x, best->frontier_y);
      const int frontier_col =
          static_cast<int>(std::floor((best->frontier_x - map.origin_x) / map.resolution));
      const int frontier_row =
          static_cast<int>(std::floor((best->frontier_y - map.origin_y) / map.resolution));
      facts.unentered_room = UnknownRoomBeyond(map, frontier_col, frontier_row) ||
                             UnenteredRoomBeyondIn(observation, *best);
      if (!AdmitExploringGoal(facts, problem.config.minimum_frontier_cells) ||
          problem.tips.Blocked(best->frontier_x, best->frontier_y)) {
        continue;
      }
      goals.push_back(*best);
      result.debug.cluster_goals.push_back({best->x, best->y, best->frontier_x, best->frontier_y,
                                            cluster_id, best->score, best->information_gain,
                                            best->frontier_cells});
    }
  }
  std::sort(goals.begin(), goals.end(),
            [](const auto& a, const auto& b) { return a.score > b.score; });
  return result;
}
}  // namespace navigation2d::frontier_internal
