#include "navigation2d/costmap/costmap_parameters.h"

#include "navigation2d/configuration/navigation_config.h"

namespace navigation2d {
CostmapParameters CostmapParametersFrom(const NavigationConfig& config) {
  return {.robot_radius = config.robot_radius,
          .inflation_radius = config.inflation_radius,
          .inflation_cost_scaling = config.inflation_cost_scaling,
          .obstacle_max_range = config.obstacle_max_range,
          .raytrace_max_range = config.raytrace_max_range,
          .local_window_width = config.local_window_width,
          .local_window_height = config.local_window_height};
}
}  // namespace navigation2d
