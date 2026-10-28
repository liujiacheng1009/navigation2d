#include "navigation2d/costmap/layered_costmap.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace navigation2d {

LayeredCostmap::LayeredCostmap(Grid2d map, NavigationConfig config)
    : LayeredCostmap(std::move(map), CostmapParametersFrom(config), ParametersTag{}) {}

LayeredCostmap LayeredCostmap::WithParameters(Grid2d map, CostmapParameters parameters) {
  return LayeredCostmap(std::move(map), parameters, ParametersTag{});
}

LayeredCostmap::LayeredCostmap(Grid2d&& map, CostmapParameters parameters, ParametersTag)
    : static_map_(std::move(map)),
      config_(parameters),
      obstacles_(static_map_.width() * static_map_.height()),
      master_(static_map_.width() * static_map_.height(), kFree) {
  Reinflate();
}

void LayeredCostmap::MarkObstacle(double x, double y) {
  obstacles_.Mark(static_map_, x, y);
  Reinflate();
}
void LayeredCostmap::ClearObstacle(double x, double y) {
  obstacles_.Clear(static_map_, x, y);
  Reinflate();
}
void LayeredCostmap::UpdateObstacleLayer(const Pose2d& pose, const LaserScan& scan) {
  if (obstacles_.Update(static_map_, pose, scan,
                        {config_.obstacle_max_range, config_.raytrace_max_range})) {
    Reinflate();
  }
}
void LayeredCostmap::UpdateObstacleLayer(const Pose2d& pose, const PointCloud2d& cloud) {
  if (obstacles_.Update(static_map_, pose, cloud,
                        {config_.obstacle_max_range, config_.raytrace_max_range})) {
    Reinflate();
  }
}
void LayeredCostmap::Reinflate() {
  const std::vector<std::uint8_t> previous = master_;
  std::fill(master_.begin(), master_.end(), kFree);
  std::vector<std::pair<int, int>> lethal;
  for (int y = 0; y < static_map_.height(); ++y) {
    for (int x = 0; x < static_map_.width(); ++x) {
      const int index = y * static_map_.width() + x;
      // A live return on an unseen cell is a real obstacle. Unknown only stays
      // unknown when nothing has marked it.
      if (obstacles_.at(index) == kLethal || static_map_.occupied(x, y)) {
        master_[index] = kLethal;
        lethal.push_back({x, y});
      } else if (static_map_.unknown(x, y)) {
        master_[index] = kUnknown;
      }
    }
  }
  const int radius =
      static_cast<int>(std::ceil(config_.inflation_radius / static_map_.resolution()));
  for (const auto& [ox, oy] : lethal) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        const int x = ox + dx, y = oy + dy;
        if (x < 0 || y < 0 || x >= static_map_.width() || y >= static_map_.height()) {
          continue;
        }
        const double distance = std::hypot(dx, dy) * static_map_.resolution();
        if (distance > config_.inflation_radius || distance == 0.) {
          continue;
        }
        const std::uint8_t value = distance <= config_.robot_radius
                                       ? static_cast<std::uint8_t>(kInscribed)
                                       : static_cast<std::uint8_t>(std::clamp(
                                             252. * std::exp(-config_.inflation_cost_scaling *
                                                             (distance - config_.robot_radius)),
                                             1., 252.));
        master_[y * static_map_.width() + x] =
            std::max(master_[y * static_map_.width() + x], value);
      }
    }
  }
  changes_.Record(previous, master_);
}
std::vector<int> LayeredCostmap::ChangedCellsSince(std::uint64_t revision) const {
  return changes_.Since(revision, master_.size());
}

std::uint8_t LayeredCostmap::cost(int x, int y) const {
  if (x < 0 || y < 0 || x >= static_map_.width() || y >= static_map_.height()) {
    return kLethal;
  }
  return master_[y * static_map_.width() + x];
}

bool LayeredCostmap::lethal(double x, double y, double radius) const {
  const auto [cx, cy] = static_map_.ToCell(x, y);
  if (cx < 0 || cy < 0 || cx >= static_map_.width() || cy >= static_map_.height()) {
    return true;
  }
  const double resolution = static_map_.resolution();
  const int cells =
      static_cast<int>(std::ceil((std::max(0., radius) + std::sqrt(.5) * resolution) / resolution));
  for (int dy = -cells; dy <= cells; ++dy) {
    for (int dx = -cells; dx <= cells; ++dx) {
      const int obstacle_x = cx + dx, obstacle_y = cy + dy;
      if (obstacle_x < 0 || obstacle_y < 0 || obstacle_x >= static_map_.width() ||
          obstacle_y >= static_map_.height()) {
        return true;
      }
      if (cost(obstacle_x, obstacle_y) != kLethal) {
        continue;
      }
      const auto [wx, wy] = static_map_.CellCenter(obstacle_x, obstacle_y);
      const double half = .5 * resolution;
      const double nearest_x = std::max(std::abs(x - wx) - half, 0.);
      const double nearest_y = std::max(std::abs(y - wy) - half, 0.);
      if (std::hypot(nearest_x, nearest_y) <= std::max(0., radius)) {
        return true;
      }
    }
  }
  return false;
}

double LayeredCostmap::nearestLethalDistance(double x, double y) const {
  double best = std::numeric_limits<double>::infinity();
  const double resolution = static_map_.resolution();
  const double half = .5 * resolution;
  for (int cy = 0; cy < static_map_.height(); ++cy) {
    for (int cx = 0; cx < static_map_.width(); ++cx) {
      if (master_[cy * static_map_.width() + cx] != kLethal) {
        continue;
      }
      const auto [wx, wy] = static_map_.CellCenter(cx, cy);
      const double nearest_x = std::max(std::abs(x - wx) - half, 0.);
      const double nearest_y = std::max(std::abs(y - wy) - half, 0.);
      best = std::min(best, std::hypot(nearest_x, nearest_y));
    }
  }
  return best;
}

std::vector<std::uint8_t> LayeredCostmap::RollingWindow(const Pose2d& pose, int* width, int* height,
                                                        int* origin_x, int* origin_y) const {
  *width = std::max(
      1, static_cast<int>(std::ceil(config_.local_window_width / static_map_.resolution())));
  *height = std::max(
      1, static_cast<int>(std::ceil(config_.local_window_height / static_map_.resolution())));
  const auto [cx, cy] = static_map_.ToCell(X(pose), Y(pose));
  *origin_x = cx - *width / 2;
  *origin_y = cy - *height / 2;
  std::vector<std::uint8_t> result(*width * *height);
  for (int y = 0; y < *height; ++y) {
    for (int x = 0; x < *width; ++x) {
      result[y * *width + x] = cost(*origin_x + x, *origin_y + y);
    }
  }
  return result;
}

std::uint64_t LayeredCostmap::digest() const {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const auto value : master_) {
    hash ^= value;
    hash *= 1099511628211ULL;
  }
  return hash;
}

}  // namespace navigation2d
