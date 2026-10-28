#include "navigation2d/exploration/frontier_travel_graph.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <span>
#include <utility>

#include "navigation2d/exploration/planning_grid_view.h"
namespace navigation2d::frontier_internal {
namespace {
struct TravelMasks {
  std::vector<unsigned char> safe, discovery;
};
TravelMasks BuildMasks(const ExplorationGrid& map, const PlanningGridView& geometry,
                       double footprint_clearance_m) {
  TravelMasks masks{std::vector<unsigned char>(map.cells.size(), 0),
                    std::vector<unsigned char>(map.cells.size(), 0)};
  for (int row = 0; row < map.height; ++row) {
    for (int col = 0; col < map.width; ++col) {
      masks.safe[static_cast<std::size_t>(row) * map.width + col] =
          geometry.SafeViewpoint(col, row);
    }
  }
  // Relaxed clearance discovers room boundaries only. Executable viewpoints
  // still require the independent SafeViewpoint mask and live dispatch checks.
  const double discovery_clearance =
      std::clamp(footprint_clearance_m - .12, .14, kPlanningRadius());
  for (int row = 0; row < map.height; ++row) {
    for (int col = 0; col < map.width; ++col) {
      const auto index = static_cast<std::size_t>(row) * map.width + col;
      masks.discovery[index] =
          geometry.Free(col, row) && geometry.ClearAt(col, row, discovery_clearance);
    }
  }
  return masks;
}
int NearestStart(const ExplorationGrid& map, std::span<const unsigned char> mask,
                 FrontierTravelOrigin origin) {
  const auto [robot_col, robot_row, radius] = origin;
  int start = -1;
  double nearest = std::numeric_limits<double>::infinity();
  for (int row = std::max(0, robot_row - radius);
       row <= std::min(map.height - 1, robot_row + radius); ++row) {
    for (int col = std::max(0, robot_col - radius);
         col <= std::min(map.width - 1, robot_col + radius); ++col) {
      const int index = row * map.width + col;
      if (!mask[index]) {
        continue;
      }
      const double distance = std::hypot(col - robot_col, row - robot_row);
      if (distance < nearest) {
        nearest = distance;
        start = index;
      }
    }
  }
  return start;
}
std::vector<int> TravelDistances(const ExplorationGrid& map, std::span<const unsigned char> mask,
                                 int start) {
  std::vector<int> distances(map.cells.size(), -1);
  if (start < 0) {
    return distances;
  }
  std::deque<int> queue{start};
  distances[start] = 0;
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  while (!queue.empty()) {
    const int current = queue.front();
    queue.pop_front();
    const int row = current / map.width, col = current % map.width;
    for (int direction = 0; direction < 4; ++direction) {
      const int x = col + dx[direction], y = row + dy[direction];
      if (x < 0 || y < 0 || x >= map.width || y >= map.height) {
        continue;
      }
      const int next = y * map.width + x;
      if (!mask[next] || distances[next] >= 0) {
        continue;
      }
      distances[next] = distances[current] + 1;
      queue.push_back(next);
    }
  }
  return distances;
}
}  // namespace
int NearestSafeTravelCell(const ExplorationGrid& map, std::span<const unsigned char> safe,
                          double world_x, double world_y) {
  return NearestStart(map, safe,
                      {static_cast<int>(std::floor((world_x - map.origin_x) / map.resolution)),
                       static_cast<int>(std::floor((world_y - map.origin_y) / map.resolution)),
                       static_cast<int>(std::ceil(1.0 / map.resolution))});
}
std::optional<FrontierTravelGraph> BuildFrontierTravelGraph(const ExplorationGrid& map,
                                                            const ExplorationGrid& observation,
                                                            double footprint_clearance_m,
                                                            double robot_x, double robot_y) {
  const PlanningGridView geometry(map, observation, footprint_clearance_m);
  auto masks = BuildMasks(map, geometry, footprint_clearance_m);
  const FrontierTravelOrigin origin{
      static_cast<int>(std::floor((robot_x - map.origin_x) / map.resolution)),
      static_cast<int>(std::floor((robot_y - map.origin_y) / map.resolution)),
      static_cast<int>(std::ceil(1.0 / map.resolution))};
  const int start = NearestStart(map, masks.safe, origin);
  if (start < 0) {
    return std::nullopt;
  }
  auto travel = TravelDistances(map, masks.safe, start);
  const int discovery_start =
      masks.discovery[start] ? start : NearestStart(map, masks.discovery, origin);
  const auto discovery = TravelDistances(map, masks.discovery, discovery_start);
  // Preserve execution distances wherever available. Discovery hints may fill
  // unreachable cells, but never replace or authorize an executable viewpoint.
  for (std::size_t index = 0; index < travel.size(); ++index) {
    if (travel[index] < 0) {
      travel[index] = discovery[index];
    }
  }
  ApplyBootstrapFrontierTravel(map, observation, origin, travel);
  return FrontierTravelGraph{std::move(masks.safe), std::move(travel)};
}
std::optional<FrontierTravelGraph> BuildSafeFrontierTravelGraph(const ExplorationGrid& map,
                                                                const ExplorationGrid& observation,
                                                                double footprint_clearance_m,
                                                                double robot_x, double robot_y) {
  const PlanningGridView geometry(map, observation, footprint_clearance_m);
  std::vector<unsigned char> safe(map.cells.size(), 0);
  for (int row = 0; row < map.height; ++row) {
    for (int col = 0; col < map.width; ++col) {
      safe[static_cast<std::size_t>(row) * map.width + col] = geometry.SafeViewpoint(col, row);
    }
  }
  const FrontierTravelOrigin origin{
      static_cast<int>(std::floor((robot_x - map.origin_x) / map.resolution)),
      static_cast<int>(std::floor((robot_y - map.origin_y) / map.resolution)),
      static_cast<int>(std::ceil(1.0 / map.resolution))};
  const int start = NearestStart(map, safe, origin);
  if (start < 0) {
    return std::nullopt;
  }
  auto travel = TravelDistances(map, safe, start);
  return FrontierTravelGraph{std::move(safe), std::move(travel)};
}

}  // namespace navigation2d::frontier_internal
