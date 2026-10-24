#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

#include "navigation2d/exploration/frontier_travel_graph.h"
#include "navigation2d/exploration/observation_grid_view.h"
namespace navigation2d::frontier_internal {
void ApplyBootstrapFrontierTravel(const ExplorationGrid& map, const ExplorationGrid& observation,
                                  FrontierTravelOrigin origin, std::vector<int>& travel_cells) {
  const ObservationGridView observed(observation);
  const auto [robot_col, robot_row, start_search] = origin;
  constexpr int cardinal_x[] = {-1, 1, 0, 0};
  constexpr int cardinal_y[] = {0, 0, -1, 1};
  // During the first scan, costmap inflation can temporarily mark every
  // viewpoint around the robot unsafe even though the raw map already has a
  // connected free corridor and visible unknown boundary.  Without a first
  // goal the robot cannot move far enough for the mapper to resolve that
  // boundary.  Use the raw free-cell graph only for this small bootstrap
  // window; mature maps continue to require the full footprint-safe graph.
  const std::size_t raw_frontier_count = observed.CountFrontiers();
  const bool bootstrap_map = observed.KnownCells() < 10000 && raw_frontier_count > 0;
  if (bootstrap_map) {
    bool reachable_raw_frontier = false;
    for (int index = 0; index < static_cast<int>(travel_cells.size()); ++index) {
      if (travel_cells[index] < 0) {
        continue;
      }
      const int row = index / map.width, col = index % map.width;
      if (observed.Frontier(col, row)) {
        reachable_raw_frontier = true;
        break;
      }
    }
    if (!reachable_raw_frontier) {
      std::fill(travel_cells.begin(), travel_cells.end(), -1);
      int raw_start = -1;
      double raw_start_distance = std::numeric_limits<double>::infinity();
      for (int row = std::max(0, robot_row - start_search);
           row <= std::min(map.height - 1, robot_row + start_search); ++row) {
        for (int col = std::max(0, robot_col - start_search);
             col <= std::min(map.width - 1, robot_col + start_search); ++col) {
          const int index = row * map.width + col;
          if (map.cells[static_cast<std::size_t>(row) * map.width + col] != 0) {
            continue;
          }
          const double distance = std::hypot(col - robot_col, row - robot_row);
          if (distance < raw_start_distance) {
            raw_start_distance = distance;
            raw_start = index;
          }
        }
      }
      if (raw_start >= 0) {
        std::deque<int> raw_reachable{raw_start};
        travel_cells[raw_start] = 0;
        while (!raw_reachable.empty()) {
          const int current = raw_reachable.front();
          raw_reachable.pop_front();
          const int row = current / map.width, col = current % map.width;
          for (int direction = 0; direction < 4; ++direction) {
            const int x = col + cardinal_x[direction], y = row + cardinal_y[direction];
            if (x < 0 || y < 0 || x >= map.width || y >= map.height) {
              continue;
            }
            const int next = y * map.width + x;
            if (travel_cells[next] >= 0 ||
                map.cells[static_cast<std::size_t>(y) * map.width + x] != 0) {
              continue;
            }
            travel_cells[next] = travel_cells[current] + 1;
            raw_reachable.push_back(next);
          }
        }
      }
    }
  }
}
}  // namespace navigation2d::frontier_internal
