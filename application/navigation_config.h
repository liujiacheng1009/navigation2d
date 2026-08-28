#pragma once

#include <string>

namespace navigation2d {

struct NavigationConfig {
  static NavigationConfig Load(const std::string& filename);
  std::string planner = "dijkstra";
  std::string controller = "rpp";
  double map_resolution = .03;
  double robot_radius = .18;
  double inflation_radius = .22;
  double inflation_cost_scaling = 8.;
  double obstacle_max_range = 5.;
  double raytrace_max_range = 5.5;
  double local_window_width = 3.;
  double local_window_height = 3.;
  double desired_linear_velocity = .24;
  double lookahead_time = 1.5;
  double min_lookahead_distance = .25;
  double max_lookahead_distance = .60;
  double rotate_to_heading_min_angle = .50;
  double min_approach_velocity = .05;
  double regulated_min_radius = .50;
  double regulated_min_speed = .08;
  double max_reverse_velocity = .12;
  double max_angular_velocity = .9;
  double max_linear_acceleration = .8;
  double max_angular_acceleration = 1.8;
  double collision_horizon = 1.;
  double max_navigation_duration = 60.;
  double control_period = .06;
  double global_replan_period = .5;
  double progress_radius = .05;
  double progress_timeout = 18.;
  double recovery_duration = 1.;
  double recovery_linear_velocity = -.08;
  double recovery_angular_velocity = .4;
  double dynamic_obstacle_radius = .20;
  int lattice_yaw_bins = 32;
  double lattice_primitive_length = .15;
  bool lattice_allow_reverse = true;
  double lattice_reverse_penalty = 1.5;
  double lattice_rotation_cost = .12;
  double lattice_cost_penalty = 2.;
  int lattice_max_expansions = 500000;
  double lattice_max_planning_time = .25;
  double goal_xy_tolerance = .05;
  double goal_yaw_tolerance = .05;
  double dwa_horizon = 1.5;
  int dwa_linear_samples = 5;
  int dwa_angular_samples = 15;
  double dwa_path_weight = 2.;
  double dwa_goal_weight = 1.;
  double dwa_obstacle_weight = 3.;
  double dwa_velocity_weight = .5;
  int mppi_time_steps = 30;
  int mppi_batch_size = 256;
  int mppi_iterations = 1;
  unsigned int mppi_seed = 20260827;
  double mppi_temperature = .3;
  double mppi_gamma = .015;
  double mppi_vx_std = .12;
  double mppi_wz_std = .35;
  double mppi_constraint_weight = 4.;
  double mppi_cost_weight = 3.81;
  double mppi_goal_weight = 5.;
  double mppi_goal_angle_weight = 3.;
  double mppi_path_align_weight = 14.;
  double mppi_path_follow_weight = 5.;
  double mppi_path_angle_weight = 2.;
  double mppi_prefer_forward_weight = 5.;
  double mppi_smoothness_weight = 1.;
  int mpc_time_steps = 20;
  std::string mpc_solver = "shooting";
  int mpc_beam_width = 36;
  double mpc_contour_weight = 12.;
  double mpc_heading_weight = 3.;
  double mpc_speed_weight = 1.;
  double mpc_control_weight = .08;
  double mpc_control_rate_weight = .04;
  double mpc_obstacle_weight = 4.;
  double mpc_progress_weight = 2.;
  double mpc_max_lateral_acceleration = .45;
  double mpc_dynamic_safety_margin = .08;
  double mpc_dynamic_sigma_scale = 2.;
};

}  // namespace navigation2d
