#include "navigation2d/costmap/obstacle_layer.h"

#include <algorithm>
#include <cmath>
namespace navigation2d::costmap_internal {
void ObstacleLayer::Mark(const Grid2d& grid, double x, double y) {
  const auto [cx, cy] = grid.ToCell(x, y);
  if (cx >= 0 && cy >= 0 && cx < grid.width() && cy < grid.height()) {
    cells_[cy * grid.width() + cx] = kLethal;
  }
}

void ObstacleLayer::Clear(const Grid2d& grid, double x, double y) {
  const auto [cx, cy] = grid.ToCell(x, y);
  if (cx >= 0 && cy >= 0 && cx < grid.width() && cy < grid.height()) {
    cells_[cy * grid.width() + cx] = kFree;
  }
}

bool ObstacleLayer::Raytrace(const Grid2d& grid, double x0, double y0, double x1, double y1,
                             bool mark_endpoint) {
  auto [x, y] = grid.ToCell(x0, y0);
  const auto [tx, ty] = grid.ToCell(x1, y1);
  const int dx = std::abs(tx - x), sx = x < tx ? 1 : -1;
  const int dy = -std::abs(ty - y), sy = y < ty ? 1 : -1;
  int error = dx + dy;
  bool changed = false;
  while (true) {
    if (x == tx && y == ty) {
      if (!mark_endpoint && x >= 0 && y >= 0 && x < grid.width() && y < grid.height()) {
        auto& value = cells_[y * grid.width() + x];
        changed = changed || value != kFree;
        value = kFree;
      }
      break;
    }
    if (x >= 0 && y >= 0 && x < grid.width() && y < grid.height()) {
      auto& value = cells_[y * grid.width() + x];
      changed = changed || value != kFree;
      value = kFree;
    }
    const int twice = 2 * error;
    if (twice >= dy) {
      error += dy;
      x += sx;
    }
    if (twice <= dx) {
      error += dx;
      y += sy;
    }
  }
  if (mark_endpoint && tx >= 0 && ty >= 0 && tx < grid.width() && ty < grid.height()) {
    auto& value = cells_[ty * grid.width() + tx];
    changed = changed || value != kLethal;
    value = kLethal;
  }
  return changed;
}

bool ObstacleLayer::Update(const Grid2d& grid, const Pose2d& pose, const LaserScan& scan,
                           ObstacleObservationLimits limits) {
  bool changed = false;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double measured = scan.ranges[i];
    if (std::isnan(measured) || measured < scan.range_min || measured < 0.) {
      continue;
    }
    // A positive infinity is a valid no-return measurement: it clears known free
    // space out to the configured raytrace limit, but never marks an obstacle.
    const bool finite_return = std::isfinite(measured);
    const bool hit =
        finite_return && measured <= std::min(scan.range_max, limits.obstacle_max_range);
    const double range =
        finite_return ? std::min(measured, limits.raytrace_max_range) : limits.raytrace_max_range;
    const double angle = Yaw(pose) + scan.angle_min + i * scan.angle_increment;
    changed = Raytrace(grid, X(pose), Y(pose), X(pose) + range * std::cos(angle),
                       Y(pose) + range * std::sin(angle), hit) ||
              changed;
  }
  return changed;
}

bool ObstacleLayer::Update(const Grid2d& grid, const Pose2d& pose, const PointCloud2d& cloud,
                           ObstacleObservationLimits limits) {
  bool changed = false;
  for (const auto& point : cloud.points) {
    const double range = point.norm();
    if (range <= 0. || range > std::min(cloud.range_max, limits.obstacle_max_range)) {
      continue;
    }
    const Eigen::Vector2d world = pose * point;
    changed = Raytrace(grid, X(pose), Y(pose), world.x(), world.y(), true) || changed;
  }
  return changed;
}

}  // namespace navigation2d::costmap_internal
