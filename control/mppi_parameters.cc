#include "navigation2d/control/mppi_parameters.h"

#include "navigation2d/configuration/navigation_config.h"

namespace navigation2d {
MppiParameters MppiParametersFrom(const NavigationConfig& config) {
  return {{config.control_period, config.desired_linear_velocity, config.max_reverse_velocity,
           config.max_angular_velocity, config.max_linear_acceleration,
           config.max_angular_acceleration, config.mppi_time_steps, config.mppi_batch_size,
           config.mppi_seed, config.mppi_temperature, config.mppi_vx_std, config.mppi_wz_std},
          DynamicSafetyParametersFrom(config),
          config.mppi_iterations,
          config.mppi_gamma,
          config.mppi_cost_weight,
          config.mppi_goal_weight,
          config.mppi_goal_angle_weight,
          config.mppi_path_align_weight,
          config.mppi_path_follow_weight,
          config.mppi_path_angle_weight,
          config.mppi_prefer_forward_weight,
          config.mppi_smoothness_weight};
}
}  // namespace navigation2d
