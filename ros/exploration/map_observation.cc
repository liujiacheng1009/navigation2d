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

}  // namespace navigation2d::exploration_ros
