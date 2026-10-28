#pragma once
#include "navigation2d/control/corridor_parameters.h"

namespace navigation2d {
struct NavigationConfig;
// Runtime inputs consumed by the generated solver adapter. Generated model
// constants remain in the generated C interface, outside this value object.
struct AcadosParameters {
  CorridorParameters corridor;
  double control_period;
  double desired_linear_velocity;
  double max_reverse_velocity;
  double max_angular_velocity;
  double min_approach_velocity;
  double robot_radius;
  double dynamic_prediction_timeout;
  double mpc_dynamic_safety_margin;
  double mpc_dynamic_sigma_scale;
  double mpc_deadline;
};
AcadosParameters AcadosParametersFrom(const NavigationConfig& config);
}  // namespace navigation2d
