#include "navigation2d/ros/exploration/map_observation.h"

#include <algorithm>
#include <cmath>
namespace navigation2d::exploration_ros {
bool SegmentCutsWall(const nav_msgs::msg::OccupancyGrid* map, double x0, double y0, double x1,
                     double y1) {
  if (!map || map->info.resolution <= 0.) {
    return false;
  }
  const double resolution = map->info.resolution;
  const double origin_x = map->info.origin.position.x;
  const double origin_y = map->info.origin.position.y;
  const int width = static_cast<int>(map->info.width);
  const int height = static_cast<int>(map->info.height);
  const double length = std::hypot(x1 - x0, y1 - y0);
  const int samples =
      std::max(2, static_cast<int>(std::ceil(length / std::max(.02, .5 * resolution))));
  for (int sample = 0; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / samples;
    const double wx = x0 + ratio * (x1 - x0);
    const double wy = y0 + ratio * (y1 - y0);
    const int col = static_cast<int>(std::floor((wx - origin_x) / resolution));
    const int row = static_cast<int>(std::floor((wy - origin_y) / resolution));
    if (col < 0 || row < 0 || col >= width || row >= height) {
      return true;
    }
    if (map->data[static_cast<std::size_t>(row) * map->info.width + col] >= 50) {
      return true;
    }
  }
  return false;
}

bool ObservedRouteCutsWall(const nav_msgs::msg::OccupancyGrid* map, const Path& route,
                           double origin_x, double origin_y,
                           const exploration::PlanningSnapshot& snapshot,
                           const std::optional<exploration::PoseObservation>& robot,
                           const std::optional<exploration::ScanEvidenceView>& scan,
                           bool execution_tick) {
  if (!map || map->info.resolution <= 0. || route.size() < 2) {
    return false;
  }
  navigation2d::ExplorationGrid live;
  live.width = static_cast<int>(map->info.width);
  live.height = static_cast<int>(map->info.height);
  live.resolution = map->info.resolution;
  live.origin_x = map->info.origin.position.x;
  live.origin_y = map->info.origin.position.y;
  live.cells.resize(static_cast<std::size_t>(live.width) * live.height);
  for (int row = 0; row < live.height; ++row) {
    for (int col = 0; col < live.width; ++col) {
      const auto cell = map->data[static_cast<std::size_t>(row) * map->info.width + col];
      std::int8_t value = 0;
      if (cell < 0) {
        value = -1;
      } else if (cell >= 50) {
        value = 100;
      }
      live.cells[static_cast<std::size_t>(row) * live.width + col] = value;
    }
  }
  std::vector<std::pair<double, double>> path_xy;
  path_xy.reserve(route.size());
  for (const auto& pose : route) {
    path_xy.emplace_back(navigation2d::X(pose) + origin_x, navigation2d::Y(pose) + origin_y);
  }
  const double robot_x = robot ? robot->x : path_xy.front().first;
  const double robot_y = robot ? robot->y : path_xy.front().second;
  return navigation2d::exploration::RouteCutsMappedObstacle(
      std::move(live), std::move(path_xy), snapshot, robot_x, robot_y, robot ? robot->yaw : 0.,
      robot ? scan : std::nullopt, execution_tick);
}
}  // namespace navigation2d::exploration_ros
