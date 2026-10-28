#pragma once

namespace navigation2d {
struct NavigationConfig;
struct DynamicSafetyParameters {
  double robot_radius;
  double mpc_dynamic_safety_margin;
  double mpc_dynamic_sigma_scale;
  double dynamic_prediction_timeout;
  double control_period;
  double collision_horizon;
  double ttc_horizon;
};
DynamicSafetyParameters DynamicSafetyParametersFrom(const NavigationConfig& config);
}  // namespace navigation2d
