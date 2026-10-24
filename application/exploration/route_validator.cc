#include "navigation2d/application/exploration/route_validator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
bool RouteCutsMappedObstacle(ExplorationGrid live, std::vector<std::pair<double, double>> path_xy,
                             const PlanningSnapshot& snapshot, double robot_x, double robot_y,
                             double robot_yaw, const std::optional<ScanEvidenceView>& scan,
                             bool execution_tick) {
  if (path_xy.size() < 2 || live.resolution <= 0.) {
    return false;
  }
  if (snapshot.free_mask.size() ==
          static_cast<std::size_t>(snapshot.grid.width) * snapshot.grid.height &&
      snapshot.grid.resolution > 0.) {
    for (int row = 0; row < live.height; ++row) {
      for (int col = 0; col < live.width; ++col) {
        auto& cell = live.cells[static_cast<std::size_t>(row) * live.width + col];
        if (cell <= 0) {
          continue;
        }
        const double wx = live.origin_x + (col + .5) * live.resolution;
        const double wy = live.origin_y + (row + .5) * live.resolution;
        const int pc =
            static_cast<int>(std::floor((wx - snapshot.grid.origin_x) / snapshot.grid.resolution));
        const int pr =
            static_cast<int>(std::floor((wy - snapshot.grid.origin_y) / snapshot.grid.resolution));
        if (pc < 0 || pr < 0 || pc >= snapshot.grid.width || pr >= snapshot.grid.height) {
          continue;
        }
        if (snapshot.free_mask[static_cast<std::size_t>(pr) * snapshot.grid.width + pc]) {
          cell = 0;
        }
      }
    }
  }
  if (execution_tick) {
    std::size_t nearest = 0;
    double nearest_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path_xy.size(); ++i) {
      const double d = std::hypot(path_xy[i].first - robot_x, path_xy[i].second - robot_y);
      if (d < nearest_d) {
        nearest_d = d;
        nearest = i;
      }
    }
    if (nearest > 0) {
      path_xy.erase(path_xy.begin(), path_xy.begin() + static_cast<std::ptrdiff_t>(nearest - 1));
    }
    if (path_xy.size() < 2) {
      return false;
    }
  }
  const auto force_free = [&](int col, int row) {
    if (col < 0 || row < 0 || col >= live.width || row >= live.height) {
      return;
    }
    auto& cell = live.cells[static_cast<std::size_t>(row) * live.width + col];
    if (cell > 0) {
      cell = 0;
    }
  };
  const auto free_disk = [&](double x, double y, double radius_m) {
    const int reach = static_cast<int>(std::ceil(radius_m / live.resolution));
    const int col = static_cast<int>(std::floor((x - live.origin_x) / live.resolution));
    const int row = static_cast<int>(std::floor((y - live.origin_y) / live.resolution));
    for (int dy = -reach; dy <= reach; ++dy) {
      for (int dx = -reach; dx <= reach; ++dx) {
        if (dx * dx + dy * dy > reach * reach) {
          continue;
        }
        force_free(col + dx, row + dy);
      }
    }
  };
  free_disk(robot_x, robot_y, navigation2d::kPlanningRadius());
  // A cell the current scan passes through is free now; a new occupied
  // mark there is mapper noise, not a wall on the route.
  if (execution_tick && scan) {
    const double yaw = robot_yaw;
    const double sensor_x = robot_x + navigation2d::kLidarForwardOffset() * std::cos(yaw);
    const double sensor_y = robot_y + navigation2d::kLidarForwardOffset() * std::sin(yaw);
    const double evidence_range = std::min(3.0, static_cast<double>(scan->range_max));
    const double step = std::max(.5 * live.resolution, .02);
    for (std::size_t beam = 0; beam < scan->ranges.size(); ++beam) {
      const double range = scan->ranges[beam];
      if (!std::isfinite(range) || range < scan->range_min || range > evidence_range) {
        continue;
      }
      const double angle =
          yaw + scan->angle_min + static_cast<double>(beam) * scan->angle_increment;
      const double usable = std::max(0., range - std::max(.06, .5 * live.resolution));
      for (double distance = 0.; distance < usable; distance += step) {
        const double x = sensor_x + distance * std::cos(angle);
        const double y = sensor_y + distance * std::sin(angle);
        const int col = static_cast<int>(std::floor((x - live.origin_x) / live.resolution));
        const int row = static_cast<int>(std::floor((y - live.origin_y) / live.resolution));
        force_free(col, row);
      }
    }
  }
  // Accepting a plan uses the planner's own circle, so a Theta* chord that
  // grazes a corner is still caught. Driving uses the chassis.
  return navigation2d::PathCutsLiveOccupied(
      live, path_xy, robot_x, robot_y,
      execution_tick ? navigation2d::kChassisRadius() : navigation2d::kPlanningRadius());
}
}  // namespace navigation2d::exploration
