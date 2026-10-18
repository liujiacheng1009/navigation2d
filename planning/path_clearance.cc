#include "navigation2d/planning/path_clearance.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "navigation2d/geometry/pose_2d.h"
#include "navigation2d/planning/collision_checker.h"

namespace navigation2d {
namespace {

constexpr double kRayMaxRangeM = 2.5;

bool CellBlocksWidth(const LayeredCostmap& costmap, int cx, int cy) {
  const Grid2d& grid = costmap.grid();
  if (cx < 0 || cy < 0 || cx >= grid.width() || cy >= grid.height()) return true;
  const std::uint8_t c = costmap.cost(cx, cy);
  return c == kLethal || c == kUnknown;
}

// Distance along (dx,dy) from (x,y) until lethal/unknown or map edge.
double RayToBlock(const LayeredCostmap& costmap, double x, double y, double dx,
                  double dy) {
  const Grid2d& grid = costmap.grid();
  const double step = std::max(grid.resolution() * .5, .02);
  double dist = 0.;
  for (; dist <= kRayMaxRangeM; dist += step) {
    const auto [cx, cy] = grid.ToCell(x + dx * dist, y + dy * dist);
    if (CellBlocksWidth(costmap, cx, cy)) return dist;
  }
  return kRayMaxRangeM;
}

double PerpendicularFreeWidth(const LayeredCostmap& costmap, double x, double y,
                              double heading_x, double heading_y) {
  const double hn = std::hypot(heading_x, heading_y);
  if (hn < 1e-9) {
    // Degenerate: fall back to isotropic estimate from distance field.
    planning_internal::DistanceField field(costmap);
    return 2. * field.distance(x, y);
  }
  const double ux = heading_x / hn;
  const double uy = heading_y / hn;
  // Left / right normals.
  const double left = RayToBlock(costmap, x, y, -uy, ux);
  const double right = RayToBlock(costmap, x, y, uy, -ux);
  return left + right;
}

}  // namespace

PathClearanceProfile AnalyzePathClearanceProfile(const Path& path,
                                                 const LayeredCostmap& costmap,
                                                 double robot_radius) {
  PathClearanceProfile profile;
  if (path.empty()) {
    profile.path_min_clearance_m = 0.;
    profile.bottleneck_min_clearance_m = 0.;
    profile.has_lethal_tip = true;
    return profile;
  }
  planning_internal::DistanceField field(costmap);
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double x = X(path[index]);
    const double y = Y(path[index]);
    double hx = 0., hy = 0.;
    if (index + 1 < path.size()) {
      hx = X(path[index + 1]) - x;
      hy = Y(path[index + 1]) - y;
    } else if (index > 0) {
      hx = x - X(path[index - 1]);
      hy = y - Y(path[index - 1]);
    }
    const double margin =
        std::max(0., field.distance(x, y) - robot_radius);
    profile.path_min_clearance_m =
        std::min(profile.path_min_clearance_m, margin);
    const double free_width = PerpendicularFreeWidth(costmap, x, y, hx, hy);
    double segment = 0.;
    if (index + 1 < path.size())
      segment = std::hypot(X(path[index + 1]) - x, Y(path[index + 1]) - y);

    if (margin < kPinchMinPathMargin) {
      profile.has_lethal_tip = true;
      continue;
    }
    if (free_width >= kOpenFreeWidthMin) {
      if (margin < kOpenPreferPathMargin) {
        profile.has_open_wall_scrape = true;
        profile.open_scrape_length_m += segment;
      }
    } else {
      profile.bottleneck_min_clearance_m =
          std::min(profile.bottleneck_min_clearance_m, margin);
    }
  }
  if (!std::isfinite(profile.bottleneck_min_clearance_m))
    profile.bottleneck_min_clearance_m = profile.path_min_clearance_m;
  return profile;
}

}  // namespace navigation2d
