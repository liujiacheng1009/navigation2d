#include "navigation2d/exploration/unvisited_space.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "navigation2d/exploration/planning_grid_view.h"
namespace navigation2d::frontier_internal {
std::vector<ExplorationGoal> UnvisitedSpaceGoalsAt(
    const ExplorationGrid& map, const ExplorationGrid& observation, double footprint_clearance,
    const std::vector<std::pair<double, double>>& trail, double visit_radius_m,
    double minimum_area_m2, const std::vector<std::pair<double, double>>& room_seeds,
    double seed_radius_m) {
  const PlanningGridView geometry(map, observation, footprint_clearance);
  std::vector<ExplorationGoal> goals;
  if (map.width <= 2 || map.height <= 2 || map.resolution <= 0. ||
      map.cells.size() != static_cast<std::size_t>(map.width * map.height) || trail.empty() ||
      room_seeds.empty()) {
    return goals;
  }
  const int n = map.width * map.height;
  const double inf = std::numeric_limits<double>::infinity();
  constexpr int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
  constexpr int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
  const auto distance = FreeSpaceDistances(map, trail, inf, 0, nullptr);
  // Probe targets sit just past a jamb and may land on a wall cell once the
  // room is mapped.
  const auto seed_distance = FreeSpaceDistances(
      map, room_seeds, seed_radius_m,
      static_cast<int>(std::ceil(kPlanningRadius() / map.resolution)), &distance);
  // Unreachable free cells (inf) are not candidates: no free path from the
  // trail reaches them, so a visit goal there would never plan.
  auto far = [&](int i) { return std::isfinite(distance[i]) && distance[i] > visit_radius_m; };
  std::vector<unsigned char> seen(n, 0);
  const double cell_area = map.resolution * map.resolution;
  for (int seed = 0; seed < n; ++seed) {
    if (seen[seed] || !far(seed)) {
      continue;
    }
    std::vector<int> component{seed};
    seen[seed] = 1;
    for (std::size_t head = 0; head < component.size(); ++head) {
      const int i = component[head];
      const int col = i % map.width, row = i / map.width;
      for (int k = 0; k < 8; ++k) {
        const int c = col + dx[k], r = row + dy[k];
        if (c < 0 || r < 0 || c >= map.width || r >= map.height) {
          continue;
        }
        const int j = r * map.width + c;
        if (!seen[j] && far(j)) {
          seen[j] = 1;
          component.push_back(j);
        }
      }
    }
    if (component.size() * cell_area < minimum_area_m2) {
      continue;
    }
    if (std::none_of(component.begin(), component.end(),
                     [&](int i) { return std::isfinite(seed_distance[i]); })) {
      continue;
    }
    std::sort(component.begin(), component.end(),
              [&](int a, int b) { return distance[a] > distance[b]; });
    for (const int i : component) {
      const int col = i % map.width, row = i / map.width;
      if (!geometry.ClearAt(col, row, kPlanningRadius())) {
        continue;
      }
      const auto [wx, wy] = geometry.CellCenter(col, row);
      ExplorationGoal goal;
      goal.x = goal.frontier_x = wx;
      goal.y = goal.frontier_y = wy;
      goal.information_gain = component.size() * cell_area;
      goals.push_back(goal);
      break;
    }
  }
  return goals;
}

}  // namespace navigation2d::frontier_internal
