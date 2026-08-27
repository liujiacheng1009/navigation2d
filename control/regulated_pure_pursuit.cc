#include "navigation2d/control/regulated_pure_pursuit.h"

#include <algorithm>
#include <cmath>

namespace navigation2d {
namespace { double Angle(double a) { return std::atan2(std::sin(a), std::cos(a)); } }

Velocity RegulatedPurePursuit::Compute(const Path& path, const Pose2d& pose, Velocity current,
                                       const LayeredCostmap& costmap) const {
  size_t nearest = 0;
  double nearest_distance = 1e30;
  for (size_t i = 0; i < path.size(); ++i) {
    const double d = std::hypot(path[i].x - pose.x, path[i].y - pose.y);
    if (d < nearest_distance) { nearest_distance = d; nearest = i; }
  }
  const double lookahead = std::clamp(std::abs(current.linear) * config_.lookahead_time,
                                      config_.min_lookahead_distance,
                                      config_.max_lookahead_distance);
  size_t carrot = nearest;
  double accumulated = 0.;
  while (carrot + 1 < path.size() && accumulated < lookahead) {
    accumulated += std::hypot(path[carrot + 1].x - path[carrot].x,
                              path[carrot + 1].y - path[carrot].y);
    ++carrot;
  }
  const double dx = path[carrot].x - pose.x, dy = path[carrot].y - pose.y;
  const double heading_error = Angle(std::atan2(dy, dx) - pose.yaw);
  Velocity target;
  if (std::abs(heading_error) > config_.rotate_to_heading_min_angle) {
    target = {0., std::clamp(2.2 * heading_error, -config_.max_angular_velocity,
                            config_.max_angular_velocity)};
  } else {
  const double distance_to_goal = std::hypot(path.back().x - pose.x, path.back().y - pose.y);
  double linear = std::min(config_.desired_linear_velocity,
                           std::max(config_.min_approach_velocity, distance_to_goal));
  linear *= std::max(.35, 1. - std::abs(heading_error));
  const double lateral = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
  const double curvature = 2. * lateral / std::max(.04, dx * dx + dy * dy);
  const double radius = std::abs(curvature) < 1e-6 ? 1e9 : std::abs(1. / curvature);
  if (radius < config_.regulated_min_radius)
    linear = std::max(config_.regulated_min_speed,
                      linear * radius / config_.regulated_min_radius);
  const auto [cx, cy] = costmap.grid().ToCell(pose.x, pose.y);
  const double normalized_cost = static_cast<double>(costmap.cost(cx, cy)) / 252.;
  linear *= std::max(.25, 1. - normalized_cost);
  target = {linear, std::clamp(linear * curvature, -config_.max_angular_velocity,
                               config_.max_angular_velocity)};
  }
  const double dv = config_.max_linear_acceleration * config_.control_period;
  const double dw = config_.max_angular_acceleration * config_.control_period;
  target.linear = std::clamp(target.linear, current.linear - dv, current.linear + dv);
  target.angular = std::clamp(target.angular, current.angular - dw, current.angular + dw);
  if (CollisionImminent(pose, target, costmap)) return {0., 0.};
  return target;
}

bool RegulatedPurePursuit::CollisionImminent(const Pose2d& pose, Velocity command,
                                             const LayeredCostmap& costmap) const {
  Pose2d projected = pose;
  for (double t = 0.; t <= config_.collision_horizon; t += config_.control_period) {
    projected.yaw = Angle(projected.yaw + command.angular * config_.control_period);
    projected.x += command.linear * std::cos(projected.yaw) * config_.control_period;
    projected.y += command.linear * std::sin(projected.yaw) * config_.control_period;
    if (costmap.lethal(projected.x, projected.y, config_.robot_radius)) return true;
  }
  return false;
}

}  // namespace navigation2d
