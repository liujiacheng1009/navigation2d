#pragma once

namespace navigation2d {
struct NavigationConfig;
// Construction-time occupancy/inflation settings, in metres (scaling in 1/m).
// Does not retain algorithm registries, controller parameters or a footprint.
struct CostmapParameters {
  double robot_radius = .18;
  double inflation_radius = .22;
  double inflation_cost_scaling = 8.;
  double obstacle_max_range = 5.;
  double raytrace_max_range = 5.5;
  double local_window_width = 3.;
  double local_window_height = 3.;
};
CostmapParameters CostmapParametersFrom(const NavigationConfig& config);
}  // namespace navigation2d
