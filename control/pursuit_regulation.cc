#include "navigation2d/control/pursuit_regulation.h"

#include <algorithm>
#include <cmath>
namespace navigation2d::pursuit_internal {
namespace {
double Angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

double Smoothstep01(double t) {
  t = std::clamp(t, 0., 1.);
  return t * t * (3. - 2. * t);
}

// Local lethal clearance — full-map nearestLethalDistance is O(WH) per tick.
double LocalLethalClearance(const LayeredCostmap& costmap, double x, double y,
                            double search_radius) {
  const auto& grid = costmap.grid();
  const double resolution = grid.resolution();
  if (resolution <= 1e-9) {
    return 0.;
  }
  const double half = .5 * resolution;
  const int span = std::max(1, static_cast<int>(std::ceil(search_radius / resolution)));
  const auto [cx, cy] = grid.ToCell(x, y);
  double best = search_radius;
  bool found = false;
  for (int dy = -span; dy <= span; ++dy) {
    for (int dx = -span; dx <= span; ++dx) {
      const int ix = cx + dx, iy = cy + dy;
      if (ix < 0 || iy < 0 || ix >= grid.width() || iy >= grid.height()) {
        continue;
      }
      if (costmap.cost(ix, iy) != kLethal) {
        continue;
      }
      const auto [wx, wy] = grid.CellCenter(ix, iy);
      const double nearest_x = std::max(std::abs(x - wx) - half, 0.);
      const double nearest_y = std::max(std::abs(y - wy) - half, 0.);
      best = std::min(best, std::hypot(nearest_x, nearest_y));
      found = true;
    }
  }
  return found ? best : search_radius;
}

bool ForwardCorridorTrusted(const LayeredCostmap& costmap, const Pose2d& pose, double robot_radius,
                            double length, double half_width) {
  const double yaw = Yaw(pose);
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double lateral_c = -s, lateral_s = c;
  constexpr int kAlong = 4;
  for (int i = 1; i <= kAlong; ++i) {
    const double d = length * (static_cast<double>(i) / kAlong);
    for (const double lat : {0., half_width, -half_width}) {
      const double x = X(pose) + d * c + lat * lateral_c;
      const double y = Y(pose) + d * s + lat * lateral_s;
      if (costmap.lethal(x, y, robot_radius)) {
        return false;
      }
    }
  }
  return true;
}

double ClearanceSpeedScale(double margin, bool forward_trusted,
                           const RegulationParameters& config) {
  const double band =
      forward_trusted ? config.regulated_soft_clearance_trusted : config.regulated_soft_clearance;
  const double floor_scale = forward_trusted ? config.regulated_clearance_min_scale_trusted
                                             : config.regulated_clearance_min_scale;
  if (band <= 1e-9 || margin >= band) {
    return 1.;
  }
  if (margin <= 0.) {
    return floor_scale;
  }
  const double s = Smoothstep01(margin / band);
  return floor_scale + (1. - floor_scale) * s;
}
}  // namespace
Twist2d TrackingTarget(const Path& path, const Pose2d& pose, const Eigen::Vector2d& carrot,
                       const LayeredCostmap& costmap, const RegulationParameters& config) {
  const double dx = carrot.x() - X(pose), dy = carrot.y() - Y(pose);
  const double heading_error = Angle(std::atan2(dy, dx) - Yaw(pose));
  if (std::abs(heading_error) > config.rotate_to_heading_min_angle) {
    return Twist2d{0., std::clamp(2.2 * heading_error, -config.max_angular_velocity,
                                  config.max_angular_velocity)};
  }
  const double distance_to_goal = (path.back().translation() - pose.translation()).norm();
  double linear = std::min(config.desired_linear_velocity,
                           std::max(config.min_approach_velocity, distance_to_goal));
  linear *= std::max(.35, 1. - std::abs(heading_error));
  const double lateral = -std::sin(Yaw(pose)) * dx + std::cos(Yaw(pose)) * dy;
  const double curvature = 2. * lateral / std::max(.04, dx * dx + dy * dy);
  const double radius = std::abs(curvature) < 1e-6 ? 1e9 : std::abs(1. / curvature);
  if (radius < config.regulated_min_radius) {
    linear = std::max(config.regulated_min_speed, linear * radius / config.regulated_min_radius);
  }
  // Soft near-field regulation uses lethal clearance, not inflated cost.
  // Inflation remains for global planning preference; hard contact is still
  // CollisionImminent. When the short forward corridor is free, side walls
  // (door frames) keep a higher speed floor instead of crawling to 0.25×.
  const double search =
      config.robot_radius +
      std::max(config.regulated_soft_clearance, config.regulated_soft_clearance_trusted) + .25;
  const double clearance = LocalLethalClearance(costmap, X(pose), Y(pose), search);
  const double margin = clearance - config.robot_radius;
  const bool forward_trusted = ForwardCorridorTrusted(costmap, pose, config.robot_radius,
                                                      config.regulated_forward_trust_length,
                                                      config.regulated_forward_trust_half_width);
  const double clearance_scale = ClearanceSpeedScale(margin, forward_trusted, config);
  if (clearance_scale < 1.) {
    linear = std::max(config.regulated_min_speed, linear * clearance_scale);
  }
  return Twist2d{linear, std::clamp(linear * curvature, -config.max_angular_velocity,
                                    config.max_angular_velocity)};
}
}  // namespace navigation2d::pursuit_internal
