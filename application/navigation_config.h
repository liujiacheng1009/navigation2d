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
  double goal_xy_tolerance = .05;
  double goal_yaw_tolerance = .05;
  double dwa_horizon = 1.5;
  int dwa_linear_samples = 5;
  int dwa_angular_samples = 15;
  double dwa_path_weight = 2.;
  double dwa_goal_weight = 1.;
  double dwa_obstacle_weight = 3.;
  double dwa_velocity_weight = .5;
};

}  // namespace navigation2d
