#include "navigation2d/application/exploration/return_geometry.h"

#include <cmath>
#include <limits>

#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
ExplorationGoal SnapReturnHome(const ExplorationGrid* map, ExplorationGoal home) {
  if (!map || map->resolution <= 0.) {
    return home;
  }
  const double resolution = map->resolution;
  const double origin_x = map->origin_x;
  const double origin_y = map->origin_y;
  const int width = static_cast<int>(map->width);
  const int height = static_cast<int>(map->height);
  auto clear_at = [&](double wx, double wy, double clearance) {
    const int col = static_cast<int>(std::floor((wx - origin_x) / resolution));
    const int row = static_cast<int>(std::floor((wy - origin_y) / resolution));
    const int reach = static_cast<int>(std::ceil(clearance / resolution));
    for (int dy = -reach; dy <= reach; ++dy) {
      for (int dx = -reach; dx <= reach; ++dx) {
        if (dx * dx + dy * dy > reach * reach) {
          continue;
        }
        const int x = col + dx, y = row + dy;
        if (x < 0 || y < 0 || x >= width || y >= height) {
          return false;
        }
        if (map->cells[static_cast<std::size_t>(y) * map->width + x] >= 50) {
          return false;
        }
      }
    }
    return true;
  };
  constexpr double kHomeClearance = .30;
  if (clear_at(home.x, home.y, kHomeClearance)) {
    return home;
  }
  // Spawn often sits beside a mapped wall. Snap to the nearest clear cell
  // inside 1 m so return planning is not rejected as goal-occupied.
  double best_x = home.x, best_y = home.y;
  double best_d = std::numeric_limits<double>::infinity();
  const int search = static_cast<int>(std::ceil(1.0 / resolution));
  const int home_col = static_cast<int>(std::floor((home.x - origin_x) / resolution));
  const int home_row = static_cast<int>(std::floor((home.y - origin_y) / resolution));
  for (int dy = -search; dy <= search; ++dy) {
    for (int dx = -search; dx <= search; ++dx) {
      const int x = home_col + dx, y = home_row + dy;
      if (x < 0 || y < 0 || x >= width || y >= height) {
        continue;
      }
      const double wx = origin_x + (x + .5) * resolution;
      const double wy = origin_y + (y + .5) * resolution;
      if (!clear_at(wx, wy, kHomeClearance)) {
        continue;
      }
      const double d = std::hypot(wx - home.x, wy - home.y);
      if (d < best_d) {
        best_d = d;
        best_x = wx;
        best_y = wy;
      }
    }
  }
  if (best_d < std::numeric_limits<double>::infinity()) {
    home.x = best_x;
    home.y = best_y;
  }
  return home;
}
ReturnStep SelectReturnStep(std::span<const std::pair<double, double>> trail,
                            const ExplorationGoal& home, ReturnPosition robot,
                            const ExplorationGoal& active_goal, bool breadcrumbs) {
  const double rx = robot.x, ry = robot.y;
  const double home_dist = std::hypot(rx - home.x, ry - home.y);
  if (home_dist < .40 || trail.empty()) {
    return {ReturnStepKind::kDirect, home};
  }
  // Nearest driven sample, then one short step back along that trail.
  // A long chord from a doorway aims through the jambs: the controller spins
  // and the same hop is replanned every second.
  std::size_t nearest = trail.size() - 1;
  double nearest_d = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < trail.size(); ++i) {
    const double d = std::hypot(trail[i].first - rx, trail[i].second - ry);
    if (d < nearest_d) {
      nearest_d = d;
      nearest = i;
    }
  }
  const double min_hop = navigation2d::kSwingClearance();
  const double max_hop = navigation2d::kOpenRetreat();
  ExplorationGoal step = home;
  bool found = false;
  double best_robot = std::numeric_limits<double>::infinity();
  for (std::size_t i = nearest; i > 0; --i) {
    const auto& sample = trail[i - 1];
    const double d_robot = std::hypot(sample.first - rx, sample.second - ry);
    if (d_robot < min_hop || d_robot > max_hop) {
      continue;
    }
    const double d_home = std::hypot(sample.first - home.x, sample.second - home.y);
    if (d_home >= home_dist) {
      continue;
    }
    if (d_robot < best_robot) {
      found = true;
      best_robot = d_robot;
      step.x = sample.first;
      step.y = sample.second;
      step.yaw = std::atan2(step.y - ry, step.x - rx);
    }
  }

  if (found && breadcrumbs && std::isfinite(active_goal.x) &&
      std::hypot(step.x - active_goal.x, step.y - active_goal.y) < kPlanningRadius()) {
    return {ReturnStepKind::kRepeated, step};
  }
  return {found ? ReturnStepKind::kBreadcrumb : ReturnStepKind::kDirect, step};
}
}  // namespace navigation2d::exploration
