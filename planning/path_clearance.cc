#include "navigation2d/planning/path_clearance.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include "navigation2d/geometry/pose_2d.h"
#include "navigation2d/planning/collision_checker.h"

namespace navigation2d {
namespace {

constexpr double kRayMaxRangeM = 2.5;
constexpr double kSampleStepM = .05;

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

// Left/right span for one normal orientation.
std::pair<double, double> SideSpans(const LayeredCostmap& costmap, double x,
                                    double y, double nx, double ny) {
  const double nn = std::hypot(nx, ny);
  if (nn < 1e-9) return {0., 0.};
  const double ux = nx / nn;
  const double uy = ny / nn;
  return {RayToBlock(costmap, x, y, ux, uy), RayToBlock(costmap, x, y, -ux, -uy)};
}

// Max free_width over perpendicular and ±30° — island corners look narrow on
// one axis but open into the room on a diagonal.
struct WidthSample {
  double free_width = 0.;
  double side_min = 0.;
};

WidthSample BestWidthSample(const LayeredCostmap& costmap, double x, double y,
                            double heading_x, double heading_y) {
  WidthSample best;
  best.free_width = -1.;
  const double hn = std::hypot(heading_x, heading_y);
  if (hn < 1e-9) {
    planning_internal::DistanceField field(costmap);
    const double d = field.distance(x, y);
    best.free_width = 2. * d;
    best.side_min = d;
    return best;
  }
  const double ux = heading_x / hn;
  const double uy = heading_y / hn;
  // Perpendicular and ±30° rotated normals.
  constexpr double kAngles[] = {0., 0.5235987755982988, -0.5235987755982988};
  for (const double ang : kAngles) {
    const double ca = std::cos(ang), sa = std::sin(ang);
    // Base normal (-uy, ux); rotate by ang in the plane.
    const double nx0 = -uy;
    const double ny0 = ux;
    const double nx = ca * nx0 - sa * ny0;
    const double ny = sa * nx0 + ca * ny0;
    const auto [a, b] = SideSpans(costmap, x, y, nx, ny);
    const double width = a + b;
    if (width > best.free_width) {
      best.free_width = width;
      best.side_min = std::min(a, b);
    }
  }
  return best;
}

void AccumulateSample(PathClearanceProfile* profile,
                      const planning_internal::DistanceField& field,
                      const LayeredCostmap& costmap, double x, double y,
                      double hx, double hy, double segment_m, double robot_radius) {
  const double margin = std::max(0., field.distance(x, y) - robot_radius);
  profile->path_min_clearance_m =
      std::min(profile->path_min_clearance_m, margin);
  const WidthSample width = BestWidthSample(costmap, x, y, hx, hy);

  if (margin < kPinchMinPathMargin) {
    profile->has_lethal_tip = true;
    return;
  }
  // One-sided hug (island corner / wall foot): not a centered door even if
  // free_width is door-sized. Require open margin.
  const bool one_sided = width.side_min < kBottleneckMinSideClearance;
  const bool open_aisle = width.free_width >= kOpenFreeWidthMin;
  if (one_sided || open_aisle) {
    if (margin < kOpenPreferPathMargin) {
      profile->has_open_wall_scrape = true;
      profile->open_scrape_length_m += segment_m;
    }
    return;
  }
  // Centered bottleneck (both flanks ≥ side floor, span < open).
  profile->bottleneck_min_clearance_m =
      std::min(profile->bottleneck_min_clearance_m, margin);
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
    const double x0 = X(path[index]);
    const double y0 = Y(path[index]);
    double hx = 0., hy = 0.;
    if (index + 1 < path.size()) {
      hx = X(path[index + 1]) - x0;
      hy = Y(path[index + 1]) - y0;
    } else if (index > 0) {
      hx = x0 - X(path[index - 1]);
      hy = y0 - Y(path[index - 1]);
    }
    // Densify along the outgoing chord so corner cuts between vertices count.
    if (index + 1 < path.size()) {
      const double len = std::hypot(hx, hy);
      const int steps = std::max(1, static_cast<int>(std::ceil(len / kSampleStepM)));
      for (int s = 0; s < steps; ++s) {
        const double t = static_cast<double>(s) / static_cast<double>(steps);
        const double x = x0 + t * hx;
        const double y = y0 + t * hy;
        AccumulateSample(&profile, field, costmap, x, y, hx, hy,
                         len / static_cast<double>(steps), robot_radius);
      }
    } else {
      AccumulateSample(&profile, field, costmap, x0, y0, hx, hy, 0.,
                       robot_radius);
    }
  }
  if (!std::isfinite(profile.bottleneck_min_clearance_m))
    profile.bottleneck_min_clearance_m = profile.path_min_clearance_m;
  return profile;
}

}  // namespace navigation2d
