#include "navigation2d/control/mpc_parameters.h"

#include "navigation2d/configuration/navigation_config.h"
#include "navigation2d/control/acados_parameters.h"

namespace navigation2d {
MpcParameters MpcParametersFrom(const NavigationConfig& config) {
  return {config.mpc_solver, config.guidance_max_candidates, config.guidance_candidate_timeout,
          DynamicSafetyParametersFrom(config)};
}
AcadosParameters AcadosParametersFrom(const NavigationConfig& config) {
  return {CorridorParametersFrom(config),
          config.control_period,
          config.desired_linear_velocity,
          config.max_reverse_velocity,
          config.max_angular_velocity,
          config.min_approach_velocity,
          config.robot_radius,
          config.dynamic_prediction_timeout,
          config.mpc_dynamic_safety_margin,
          config.mpc_dynamic_sigma_scale,
          config.mpc_deadline};
}
}  // namespace navigation2d
