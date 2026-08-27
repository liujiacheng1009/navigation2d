#include "navigation2d/control/dwa_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation2d {
namespace {
double Angle(double value) { return std::atan2(std::sin(value), std::cos(value)); }
Pose2d Step(Pose2d pose, Velocity velocity, double dt) {
  pose.yaw = Angle(pose.yaw + velocity.angular * dt);
  pose.x += velocity.linear * std::cos(pose.yaw) * dt;
  pose.y += velocity.linear * std::sin(pose.yaw) * dt;
  return pose;
}
}

Velocity DwaController::Compute(const Path& path, const Pose2d& pose, Velocity current,
                                const LayeredCostmap& costmap) const {
  Velocity best{};
  double best_score = std::numeric_limits<double>::infinity();
  const double min_v = std::max(0., current.linear - config_.max_linear_acceleration * config_.dwa_horizon);
  const double max_v = std::min(config_.desired_linear_velocity,
      current.linear + config_.max_linear_acceleration * config_.dwa_horizon);
  const double min_w = std::max(-config_.max_angular_velocity,
      current.angular - config_.max_angular_acceleration * config_.dwa_horizon);
  const double max_w = std::min(config_.max_angular_velocity,
      current.angular + config_.max_angular_acceleration * config_.dwa_horizon);
  for (int vi = 0; vi < config_.dwa_linear_samples; ++vi) {
    const double v = config_.dwa_linear_samples == 1 ? max_v :
        min_v + (max_v - min_v) * vi / (config_.dwa_linear_samples - 1);
    for (int wi = 0; wi < config_.dwa_angular_samples; ++wi) {
      const double w = config_.dwa_angular_samples == 1 ? 0. :
          min_w + (max_w - min_w) * wi / (config_.dwa_angular_samples - 1);
      Pose2d projected = pose;
      double obstacle_cost = 0.; bool collision = false;
      for (double t = 0.; t < config_.dwa_horizon; t += config_.control_period) {
        projected = Step(projected, {v, w}, config_.control_period);
        if (costmap.lethal(projected.x, projected.y, config_.robot_radius)) { collision = true; break; }
        const auto [cx, cy] = costmap.grid().ToCell(projected.x, projected.y);
        obstacle_cost = std::max(obstacle_cost, static_cast<double>(costmap.cost(cx, cy)) / 252.);
      }
      if (collision) continue;
      double path_distance = std::numeric_limits<double>::infinity();
      for (const auto& point : path)
        path_distance = std::min(path_distance, std::hypot(point.x - projected.x, point.y - projected.y));
      const double goal_distance = std::hypot(path.back().x - projected.x, path.back().y - projected.y);
      const double score = config_.dwa_path_weight * path_distance +
          config_.dwa_goal_weight * goal_distance + config_.dwa_obstacle_weight * obstacle_cost -
          config_.dwa_velocity_weight * v;
      if (score < best_score) { best_score = score; best = {v, w}; }
    }
  }
  return best;
}

bool DwaController::CollisionImminent(const Pose2d& pose, Velocity command,
                                      const LayeredCostmap& costmap) const {
  Pose2d projected = pose;
  for (double t = 0.; t < config_.collision_horizon; t += config_.control_period) {
    projected = Step(projected, command, config_.control_period);
    if (costmap.lethal(projected.x, projected.y, config_.robot_radius)) return true;
  }
  return false;
}

}  // namespace navigation2d
