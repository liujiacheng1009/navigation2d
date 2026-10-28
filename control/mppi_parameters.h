#pragma once

#include "navigation2d/control/dynamic_safety_parameters.h"

namespace navigation2d {
// The numerical optimizer owns only rollout limits and sampling inputs.
struct MppiRolloutParameters {
  double control_period;
  double desired_linear_velocity;
  double max_reverse_velocity;
  double max_angular_velocity;
  double max_linear_acceleration;
  double max_angular_acceleration;
  int mppi_time_steps;
  int mppi_batch_size;
  unsigned int mppi_seed;
  double mppi_temperature;
  double mppi_vx_std;
  double mppi_wz_std;
};
struct MppiParameters {
  MppiRolloutParameters rollout;
  DynamicSafetyParameters safety;
  int mppi_iterations;
  double mppi_gamma;
  double mppi_cost_weight;
  double mppi_goal_weight;
  double mppi_goal_angle_weight;
  double mppi_path_align_weight;
  double mppi_path_follow_weight;
  double mppi_path_angle_weight;
  double mppi_prefer_forward_weight;
  double mppi_smoothness_weight;
};
MppiParameters MppiParametersFrom(const NavigationConfig& config);
}  // namespace navigation2d
