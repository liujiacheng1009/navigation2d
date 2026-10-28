#pragma once

namespace navigation2d {
struct NavigationConfig;
namespace planning_internal {
// Immutable smoothing input; conversion retains the existing validation boundary.
struct SmootherParameters {
  bool smoother_enabled;
  int smoother_max_iterations;
  double smoother_data_weight;
  double smoother_smooth_weight;
  double smoother_obstacle_weight;
  double smoother_min_clearance;
  double smoother_max_deviation;
  double smoother_max_curvature;
  double smoother_max_step;
  double smoother_tolerance;
};
SmootherParameters SmootherParametersFrom(const NavigationConfig& config);
}  // namespace planning_internal
}  // namespace navigation2d
