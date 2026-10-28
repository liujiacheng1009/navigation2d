#pragma once

namespace navigation2d {
struct NavigationConfig;
// DWA's immutable value input. Shared YAML/ROS validation belongs to the
// configuration boundary; unrelated planner/controller settings are not owned.
struct DwaParameters {
  double robot_radius;
  double desired_linear_velocity;
  double max_angular_velocity;
  double max_linear_acceleration;
  double max_angular_acceleration;
  double collision_horizon;
  double control_period;
  double dwa_horizon;
  int dwa_linear_samples;
  int dwa_angular_samples;
  double dwa_path_weight;
  double dwa_goal_weight;
  double dwa_obstacle_weight;
  double dwa_velocity_weight;
};
DwaParameters DwaParametersFrom(const NavigationConfig& config);
}  // namespace navigation2d
