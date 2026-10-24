#include "navigation2d/configuration/navigation_validation.h"

#include <stdexcept>
namespace navigation2d::configuration_detail {
void ValidateNavigationBounds(const NavigationConfig& c) {
  if (c.map_resolution <= 0. || c.robot_radius <= 0. || c.inflation_radius < c.robot_radius ||
      c.inflation_cost_scaling <= 0. || c.obstacle_max_range <= 0. ||
      c.raytrace_max_range < c.obstacle_max_range || c.local_window_width <= 0. ||
      c.local_window_height <= 0. || (!c.footprint.empty() && c.footprint.size() < 3) ||
      c.lattice_yaw_bins < 8 || c.lattice_yaw_bins % 2 != 0 ||
      c.lattice_primitive_length < c.map_resolution || c.lattice_reverse_penalty < 1. ||
      c.lattice_rotation_cost <= 0. || c.lattice_cost_penalty < 0. ||
      c.lattice_max_expansions < 1 || c.lattice_max_planning_time <= 0. ||
      c.lattice_initial_heuristic_weight < 1. || c.lattice_heuristic_weight_decrement <= 0. ||
      c.smoother_max_iterations < 1 || c.smoother_data_weight < 0. ||
      c.smoother_smooth_weight < 0. || c.smoother_obstacle_weight < 0. ||
      c.smoother_min_clearance < c.robot_radius || c.smoother_max_deviation < 0. ||
      c.smoother_max_curvature <= 0. || c.smoother_max_step <= 0. || c.smoother_tolerance <= 0. ||
      c.control_period <= 0. || c.global_replan_period < c.control_period ||
      c.mppi_time_steps < 2 || c.mppi_batch_size < 2 || c.mppi_iterations < 1 ||
      c.mppi_temperature <= 0. || c.mppi_vx_std <= 0. || c.mppi_wz_std <= 0. ||
      (c.mpc_solver != "mppi" && c.mpc_solver != "acados") || c.mpc_time_steps < 2 ||
      c.mpc_contour_weight < 0. || c.mpc_heading_weight < 0. || c.mpc_speed_weight < 0. ||
      c.mpc_control_weight < 0. || c.mpc_control_rate_weight < 0. || c.mpc_obstacle_weight < 0. ||
      c.mpc_progress_weight < 0. || c.mpc_max_lateral_acceleration <= 0. ||
      c.mpc_dynamic_safety_margin < 0. || c.mpc_dynamic_sigma_scale < 0. || c.mpc_deadline <= 0. ||
      c.dynamic_prediction_timeout <= 0. || c.guidance_max_candidates < 1 ||
      c.guidance_candidate_timeout <= 0. || c.collision_monitor_stop_distance < 0. ||
      c.collision_monitor_slowdown_distance <= c.collision_monitor_stop_distance ||
      c.collision_monitor_slowdown_ratio <= 0. || c.collision_monitor_slowdown_ratio > 1. ||
      c.collision_monitor_approach_horizon <= 0. || c.collision_monitor_source_timeout <= 0. ||
      c.collision_monitor_trigger_cycles < 1 || c.collision_monitor_release_cycles < 1 ||
      c.dwa_horizon <= 0. || c.dwa_linear_samples < 1 || c.dwa_angular_samples < 1) {
    throw std::runtime_error("invalid navigation configuration bounds");
  }
}
}  // namespace navigation2d::configuration_detail
