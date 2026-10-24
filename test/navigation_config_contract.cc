#include <iomanip>
#include <iostream>
#include <string>

#include "navigation2d/application/navigation_config.h"
namespace {
template <class T>
void Emit(const char* key, T value) {
  std::cout << std::quoted(key) << ':' << value << ',';
}
void Emit(const char* key, const std::string& value) {
  std::cout << std::quoted(key) << ':' << std::quoted(value) << ',';
}
}  // namespace
int main(int argc, char** argv) {
  if (argc != 2) {
    return 2;
  }
  const auto config = navigation2d::NavigationConfig::Load(argv[1]);
  std::cout << std::setprecision(17) << '{';
#define FIELD(name) Emit(#name, config.name)
  FIELD(planner);
  FIELD(controller);
  FIELD(map_resolution);
  FIELD(robot_radius);
  FIELD(inflation_radius);
  FIELD(inflation_cost_scaling);
  FIELD(obstacle_max_range);
  FIELD(raytrace_max_range);
  FIELD(local_window_width);
  FIELD(local_window_height);
  FIELD(desired_linear_velocity);
  FIELD(lookahead_time);
  FIELD(min_lookahead_distance);
  FIELD(max_lookahead_distance);
  FIELD(rotate_to_heading_min_angle);
  FIELD(min_approach_velocity);
  FIELD(regulated_min_radius);
  FIELD(regulated_min_speed);
  FIELD(regulated_soft_clearance);
  FIELD(regulated_soft_clearance_trusted);
  FIELD(regulated_clearance_min_scale);
  FIELD(regulated_clearance_min_scale_trusted);
  FIELD(regulated_forward_trust_length);
  FIELD(regulated_forward_trust_half_width);
  FIELD(max_reverse_velocity);
  FIELD(max_angular_velocity);
  FIELD(max_linear_acceleration);
  FIELD(max_angular_acceleration);
  FIELD(collision_horizon);
  FIELD(collision_monitor_enabled);
  FIELD(collision_monitor_stop_distance);
  FIELD(collision_monitor_slowdown_distance);
  FIELD(collision_monitor_slowdown_ratio);
  FIELD(collision_monitor_approach_horizon);
  FIELD(collision_monitor_source_timeout);
  FIELD(collision_monitor_trigger_cycles);
  FIELD(collision_monitor_release_cycles);
  FIELD(collision_monitor_passage_mode);
  FIELD(max_navigation_duration);
  FIELD(control_period);
  FIELD(global_replan_period);
  FIELD(progress_radius);
  FIELD(progress_timeout);
  FIELD(recovery_duration);
  FIELD(recovery_linear_velocity);
  FIELD(recovery_angular_velocity);
  FIELD(max_recovery_attempts);
  FIELD(dynamic_obstacle_radius);
  FIELD(lattice_yaw_bins);
  FIELD(lattice_primitive_length);
  FIELD(lattice_allow_reverse);
  FIELD(lattice_reverse_penalty);
  FIELD(lattice_rotation_cost);
  FIELD(lattice_cost_penalty);
  FIELD(lattice_max_expansions);
  FIELD(lattice_max_planning_time);
  FIELD(lattice_initial_heuristic_weight);
  FIELD(lattice_heuristic_weight_decrement);
  FIELD(smoother_enabled);
  FIELD(smoother_max_iterations);
  FIELD(smoother_data_weight);
  FIELD(smoother_smooth_weight);
  FIELD(smoother_obstacle_weight);
  FIELD(smoother_min_clearance);
  FIELD(smoother_max_deviation);
  FIELD(smoother_max_curvature);
  FIELD(smoother_max_step);
  FIELD(smoother_tolerance);
  FIELD(goal_xy_tolerance);
  FIELD(goal_yaw_tolerance);
  FIELD(dwa_horizon);
  FIELD(dwa_linear_samples);
  FIELD(dwa_angular_samples);
  FIELD(dwa_path_weight);
  FIELD(dwa_goal_weight);
  FIELD(dwa_obstacle_weight);
  FIELD(dwa_velocity_weight);
  FIELD(mppi_time_steps);
  FIELD(mppi_batch_size);
  FIELD(mppi_iterations);
  FIELD(mppi_temperature);
  FIELD(mppi_gamma);
  FIELD(mppi_vx_std);
  FIELD(mppi_wz_std);
  FIELD(mppi_constraint_weight);
  FIELD(mppi_cost_weight);
  FIELD(mppi_goal_weight);
  FIELD(mppi_goal_angle_weight);
  FIELD(mppi_path_align_weight);
  FIELD(mppi_path_follow_weight);
  FIELD(mppi_path_angle_weight);
  FIELD(mppi_prefer_forward_weight);
  FIELD(mppi_smoothness_weight);
  FIELD(mpc_time_steps);
  FIELD(mpc_solver);
  FIELD(mpc_contour_weight);
  FIELD(mpc_heading_weight);
  FIELD(mpc_speed_weight);
  FIELD(mpc_control_weight);
  FIELD(mpc_control_rate_weight);
  FIELD(mpc_obstacle_weight);
  FIELD(mpc_progress_weight);
  FIELD(mpc_max_lateral_acceleration);
  FIELD(mpc_dynamic_safety_margin);
  FIELD(mpc_dynamic_sigma_scale);
  FIELD(mpc_deadline);
  FIELD(dynamic_prediction_timeout);
  FIELD(guidance_max_candidates);
  FIELD(guidance_candidate_timeout);
#undef FIELD
  std::cout << "\"footprint\":[";
  bool first = true;
  for (const auto& point : config.footprint) {
    if (!first) {
      std::cout << ',';
    }
    std::cout << '[' << point.x() << ',' << point.y() << ']';
    first = false;
  }
  std::cout << "]}\n";
}
