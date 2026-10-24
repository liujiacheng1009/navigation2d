#include "navigation2d/planning/path_metrics.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "navigation2d/planning/collision_checker.h"

namespace navigation2d::planning_internal {
void AddPathMetrics(const Path& path, const LayeredCostmap& costmap, double robot_radius,
                    GlobalPlanningDiagnostics* diagnostics) {
  // Occupied cells only. Unknown is the frontier being approached; seeding
  // it here floored path_min_clearance_m to 0 and every doorway was a scrape.
  planning_internal::DistanceField field(costmap, /*unknown_is_obstacle=*/false);
  diagnostics->path_min_clearance_m = std::numeric_limits<double>::infinity();
  diagnostics->path_max_curvature = 0.;
  // Sample densified chords — vertex-only mins missed corner cuts between
  // waypoints that then wedged the chassis on a static map.
  constexpr double kClearanceSampleM = .05;
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double x0 = X(path[index]);
    const double y0 = Y(path[index]);
    diagnostics->path_min_clearance_m = std::min(
        diagnostics->path_min_clearance_m, std::max(0., field.distance(x0, y0) - robot_radius));
    if (index + 1 >= path.size()) {
      continue;
    }
    const double hx = X(path[index + 1]) - x0;
    const double hy = Y(path[index + 1]) - y0;
    const double len = std::hypot(hx, hy);
    const int steps = std::max(1, static_cast<int>(std::ceil(len / kClearanceSampleM)));
    for (int s = 1; s < steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(steps);
      diagnostics->path_min_clearance_m =
          std::min(diagnostics->path_min_clearance_m,
                   std::max(0., field.distance(x0 + t * hx, y0 + t * hy) - robot_radius));
    }
  }
  for (std::size_t index = 1; index + 1 < path.size(); ++index) {
    const auto first = path[index].translation() - path[index - 1].translation();
    const auto second = path[index + 1].translation() - path[index].translation();
    const double a = first.norm(), b = second.norm();
    const double chord = (path[index + 1].translation() - path[index - 1].translation()).norm();
    if (a > 1e-8 && b > 1e-8 && chord > 1e-8) {
      diagnostics->path_max_curvature = std::max(
          diagnostics->path_max_curvature,
          2. * std::abs(first.x() * second.y() - first.y() * second.x()) / (a * b * chord));
    }
  }
}
}  // namespace navigation2d::planning_internal
