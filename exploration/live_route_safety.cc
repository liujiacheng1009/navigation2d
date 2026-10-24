#include "navigation2d/exploration/live_route_safety.h"

#include <algorithm>
#include <cmath>
namespace navigation2d {
bool PreferTipAbandonAfterPathCut(int mapped_obstacle_rejects, double path_min_clearance_m) {
  (void)path_min_clearance_m;
  // Only leave after repeated live-occupied cuts. A single low-clearance
  // chord near a door jamb must replan, not abandon the room.
  return mapped_obstacle_rejects >= 2;
}
bool PathCutsLiveOccupied(const ExplorationGrid& live,
                          const std::vector<std::pair<double, double>>& path_xy, double robot_x,
                          double robot_y, double body_radius, double under_robot_m) {
  if (live.resolution <= 0. || live.width <= 2 || live.height <= 2 ||
      live.cells.size() != static_cast<std::size_t>(live.width) * live.height ||
      path_xy.size() < 2 || body_radius <= 0.) {
    return false;
  }
  const double resolution = live.resolution;
  const int width = live.width;
  const int height = live.height;
  const auto lethal = [&](int col, int row) {
    if (col < 0 || row < 0 || col >= width || row >= height) {
      return true;
    }
    return live.cells[static_cast<std::size_t>(row) * width + col] > 0;
  };
  const auto hits_body = [&](double world_x, double world_y) {
    if (std::hypot(world_x - robot_x, world_y - robot_y) < under_robot_m) {
      return false;
    }
    const int col = static_cast<int>(std::floor((world_x - live.origin_x) / resolution));
    const int row = static_cast<int>(std::floor((world_y - live.origin_y) / resolution));
    const int reach =
        static_cast<int>(std::ceil((body_radius + std::sqrt(.5) * resolution) / resolution));
    for (int dy = -reach; dy <= reach; ++dy) {
      for (int dx = -reach; dx <= reach; ++dx) {
        const int x = col + dx, y = row + dy;
        if (!lethal(x, y)) {
          continue;
        }
        const double center_x = live.origin_x + (x + .5) * resolution;
        const double center_y = live.origin_y + (y + .5) * resolution;
        const double half = .5 * resolution;
        const double nearest_x = std::max(std::abs(world_x - center_x) - half, 0.);
        const double nearest_y = std::max(std::abs(world_y - center_y) - half, 0.);
        if (std::hypot(nearest_x, nearest_y) <= body_radius) {
          return true;
        }
      }
    }
    return false;
  };
  const auto hits_centerline = [&](double world_x, double world_y) {
    if (std::hypot(world_x - robot_x, world_y - robot_y) < under_robot_m) {
      return false;
    }
    const int col = static_cast<int>(std::floor((world_x - live.origin_x) / resolution));
    const int row = static_cast<int>(std::floor((world_y - live.origin_y) / resolution));
    if (col < 0 || row < 0 || col >= width || row >= height) {
      return true;
    }
    return lethal(col, row);
  };
  for (std::size_t index = 0; index < path_xy.size(); ++index) {
    const double x = path_xy[index].first;
    const double y = path_xy[index].second;
    if (hits_body(x, y) || hits_centerline(x, y)) {
      return true;
    }
    if (index == 0) {
      continue;
    }
    const double x0 = path_xy[index - 1].first;
    const double y0 = path_xy[index - 1].second;
    const double length = std::hypot(x - x0, y - y0);
    const int samples =
        std::max(2, static_cast<int>(std::ceil(length / std::max(.01, .35 * resolution))));
    for (int sample = 1; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / samples;
      const double wx = x0 + ratio * (x - x0);
      const double wy = y0 + ratio * (y - y0);
      if (hits_body(wx, wy) || hits_centerline(wx, wy)) {
        return true;
      }
    }
  }
  return false;
}
}  // namespace navigation2d
