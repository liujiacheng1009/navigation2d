#include "navigation2d/control/dynamic_safety.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/configuration/navigation_config.h"

namespace navigation2d {
DynamicSafetyParameters DynamicSafetyParametersFrom(const NavigationConfig& config) {
  return {config.robot_radius, config.mpc_dynamic_safety_margin,
          config.mpc_dynamic_sigma_scale, config.dynamic_prediction_timeout,
          config.control_period, config.collision_horizon,
          std::max(config.collision_horizon, config.mpc_time_steps * config.control_period)};
}
bool DynamicCollisionAt(const Pose2d& pose, double time,
                        const std::vector<PredictedObstacle>& obstacles,
                        const NavigationConfig& config) {
  return DynamicCollisionAt(pose, time, obstacles, DynamicSafetyParametersFrom(config));
}
bool DynamicCollisionImminent(const Pose2d& pose, Twist2d command,
                              const std::vector<PredictedObstacle>& obstacles,
                              const NavigationConfig& config) {
  return DynamicCollisionImminent(pose, command, obstacles, DynamicSafetyParametersFrom(config));
}
double MinimumDynamicTtc(const Pose2d& pose, Twist2d command,
                         const std::vector<PredictedObstacle>& obstacles,
                         const NavigationConfig& config) {
  return MinimumDynamicTtc(pose, command, obstacles, DynamicSafetyParametersFrom(config));
}
namespace {
Pose2d Integrate(const Pose2d& pose, const Twist2d& control, double dt) {
  const double yaw = Yaw(pose);
  const double next_yaw = std::atan2(std::sin(yaw + control.angular * dt),
                                     std::cos(yaw + control.angular * dt));
  if (std::abs(control.angular) < 1e-9)
    return MakePose2d(X(pose) + control.linear * std::cos(yaw) * dt,
                      Y(pose) + control.linear * std::sin(yaw) * dt, next_yaw);
  const double radius = control.linear / control.angular;
  return MakePose2d(X(pose) + radius * (std::sin(next_yaw) - std::sin(yaw)),
                    Y(pose) - radius * (std::cos(next_yaw) - std::cos(yaw)), next_yaw);
}
}

bool DynamicCollisionAt(const Pose2d& pose, double time,
                        const std::vector<PredictedObstacle>& obstacles,
                        const DynamicSafetyParameters& config) {
  for (const auto& obstacle : obstacles) {
    if (obstacle.age_s < 0. || obstacle.age_s > config.dynamic_prediction_timeout) continue;
    const double radius = config.robot_radius + obstacle.radius + config.mpc_dynamic_safety_margin;
    const double rx = radius + config.mpc_dynamic_sigma_scale * std::max(0., obstacle.sigma_x);
    const double ry = radius + config.mpc_dynamic_sigma_scale * std::max(0., obstacle.sigma_y);
    const double prediction_time = time + obstacle.age_s;
    const double dx = X(pose) - (obstacle.x + obstacle.vx * prediction_time);
    const double dy = Y(pose) - (obstacle.y + obstacle.vy * prediction_time);
    if (dx * dx / (rx * rx) + dy * dy / (ry * ry) <= 1.) return true;
  }
  return false;
}

bool DynamicCollisionImminent(const Pose2d& pose, Twist2d command,
                              const std::vector<PredictedObstacle>& obstacles,
                              const DynamicSafetyParameters& config) {
  Pose2d projected = pose;
  for (double time = config.control_period; time <= config.collision_horizon + 1e-9;
       time += config.control_period) {
    projected = Integrate(projected, command, config.control_period);
    if (DynamicCollisionAt(projected, time, obstacles, config)) return true;
  }
  return false;
}

double MinimumDynamicTtc(const Pose2d& pose, Twist2d command,
                         const std::vector<PredictedObstacle>& obstacles,
                         const DynamicSafetyParameters& config) {
  Pose2d projected = pose;
  const double horizon = config.ttc_horizon;
  for (double time = config.control_period; time <= horizon + 1e-9;
       time += config.control_period) {
    projected = Integrate(projected, command, config.control_period);
    if (DynamicCollisionAt(projected, time, obstacles, config)) return time;
  }
  return 0.;
}

}  // namespace navigation2d
