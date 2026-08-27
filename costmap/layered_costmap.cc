#include "navigation2d/costmap/layered_costmap.h"

#include <algorithm>
#include <cmath>

namespace navigation2d {

LayeredCostmap::LayeredCostmap(Grid2d map, NavigationConfig config)
    : static_map_(std::move(map)), config_(config),
      obstacles_(static_map_.width() * static_map_.height(), kFree), master_(obstacles_.size(), kFree) {
  Reinflate();
}

void LayeredCostmap::MarkObstacle(double x, double y) {
  const auto [cx, cy] = static_map_.ToCell(x, y);
  if (cx >= 0 && cy >= 0 && cx < static_map_.width() && cy < static_map_.height())
    obstacles_[cy * static_map_.width() + cx] = kLethal;
  Reinflate();
}

void LayeredCostmap::ClearObstacle(double x, double y) {
  const auto [cx, cy] = static_map_.ToCell(x, y);
  if (cx >= 0 && cy >= 0 && cx < static_map_.width() && cy < static_map_.height())
    obstacles_[cy * static_map_.width() + cx] = kFree;
  Reinflate();
}

bool LayeredCostmap::Raytrace(double x0, double y0, double x1, double y1, bool mark_endpoint) {
  auto [x, y] = static_map_.ToCell(x0, y0);
  const auto [tx, ty] = static_map_.ToCell(x1, y1);
  const int dx = std::abs(tx - x), sx = x < tx ? 1 : -1;
  const int dy = -std::abs(ty - y), sy = y < ty ? 1 : -1;
  int error = dx + dy;
  bool changed = false;
  while (true) {
    if (x == tx && y == ty) break;
    if (x >= 0 && y >= 0 && x < static_map_.width() && y < static_map_.height()) {
      auto& value = obstacles_[y * static_map_.width() + x];
      changed = changed || value != kFree; value = kFree;
    }
    const int twice = 2 * error;
    if (twice >= dy) { error += dy; x += sx; }
    if (twice <= dx) { error += dx; y += sy; }
  }
  if (mark_endpoint && tx >= 0 && ty >= 0 && tx < static_map_.width() && ty < static_map_.height()) {
    auto& value = obstacles_[ty * static_map_.width() + tx];
    changed = changed || value != kLethal; value = kLethal;
  }
  return changed;
}

void LayeredCostmap::UpdateObstacleLayer(const Pose2d& pose, const LaserScan& scan) {
  bool changed = false;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double measured = scan.ranges[i];
    if (!std::isfinite(measured) || measured < scan.range_min) continue;
    const bool hit = measured <= std::min(scan.range_max, config_.obstacle_max_range);
    const double range = std::min(measured, config_.raytrace_max_range);
    const double angle = pose.yaw + scan.angle_min + i * scan.angle_increment;
    changed = Raytrace(pose.x, pose.y, pose.x + range * std::cos(angle),
                       pose.y + range * std::sin(angle), hit) || changed;
  }
  if (changed) Reinflate();
}

void LayeredCostmap::UpdateObstacleLayer(const Pose2d& pose, const PointCloud2d& cloud) {
  bool changed = false;
  for (const auto& point : cloud.points) {
    const double range = std::hypot(point.x, point.y);
    if (range <= 0. || range > std::min(cloud.range_max, config_.obstacle_max_range)) continue;
    const double world_x = pose.x + std::cos(pose.yaw) * point.x - std::sin(pose.yaw) * point.y;
    const double world_y = pose.y + std::sin(pose.yaw) * point.x + std::cos(pose.yaw) * point.y;
    changed = Raytrace(pose.x, pose.y, world_x, world_y, true) || changed;
  }
  if (changed) Reinflate();
}

void LayeredCostmap::Reinflate() {
  std::fill(master_.begin(), master_.end(), kFree);
  std::vector<std::pair<int, int>> lethal;
  for (int y = 0; y < static_map_.height(); ++y) for (int x = 0; x < static_map_.width(); ++x) {
    if (static_map_.occupied(x, y) || obstacles_[y * static_map_.width() + x] == kLethal) {
      master_[y * static_map_.width() + x] = kLethal; lethal.push_back({x, y});
    }
  }
  const int radius = static_cast<int>(std::ceil(config_.inflation_radius / static_map_.resolution()));
  for (const auto& [ox, oy] : lethal) for (int dy = -radius; dy <= radius; ++dy)
    for (int dx = -radius; dx <= radius; ++dx) {
      const int x = ox + dx, y = oy + dy;
      if (x < 0 || y < 0 || x >= static_map_.width() || y >= static_map_.height()) continue;
      const double distance = std::hypot(dx, dy) * static_map_.resolution();
      if (distance > config_.inflation_radius || distance == 0.) continue;
      const std::uint8_t value = distance <= config_.robot_radius ? static_cast<std::uint8_t>(kInscribed) : static_cast<std::uint8_t>(
          std::clamp(252. * std::exp(-config_.inflation_cost_scaling * (distance - config_.robot_radius)), 1., 252.));
      master_[y * static_map_.width() + x] = std::max(master_[y * static_map_.width() + x], value);
    }
}

std::uint8_t LayeredCostmap::cost(int x, int y) const {
  if (x < 0 || y < 0 || x >= static_map_.width() || y >= static_map_.height()) return kLethal;
  return master_[y * static_map_.width() + x];
}

bool LayeredCostmap::lethal(double x, double y, double radius) const {
  (void)radius;
  const auto [cx, cy] = static_map_.ToCell(x, y);
  // Inflation has already converted footprint clearance into centre-cell cost.
  return cost(cx, cy) >= kInscribed;
}

bool LayeredCostmap::PathBlocked(const Path& path, std::size_t begin) const {
  for (std::size_t i = begin; i < path.size(); ++i)
    if (lethal(path[i].x, path[i].y, config_.robot_radius)) return true;
  return false;
}

std::vector<std::uint8_t> LayeredCostmap::RollingWindow(const Pose2d& pose, int* width, int* height,
                                                        int* origin_x, int* origin_y) const {
  *width = std::max(1, static_cast<int>(std::ceil(config_.local_window_width / static_map_.resolution())));
  *height = std::max(1, static_cast<int>(std::ceil(config_.local_window_height / static_map_.resolution())));
  const auto [cx, cy] = static_map_.ToCell(pose.x, pose.y);
  *origin_x = cx - *width / 2; *origin_y = cy - *height / 2;
  std::vector<std::uint8_t> result(*width * *height);
  for (int y = 0; y < *height; ++y) for (int x = 0; x < *width; ++x)
    result[y * *width + x] = cost(*origin_x + x, *origin_y + y);
  return result;
}

std::uint64_t LayeredCostmap::digest() const {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const auto value : master_) { hash ^= value; hash *= 1099511628211ULL; }
  return hash;
}

}  // namespace navigation2d
