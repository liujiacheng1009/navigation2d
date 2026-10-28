#pragma once

#include <Eigen/Core>
#include <vector>

namespace navigation2d {
struct NavigationConfig;

// Immutable-at-construction safety settings. Owns its footprint in robot-frame
// metres; unrelated planning/controller settings are deliberately excluded.
struct CollisionMonitorParameters {
  double map_resolution = .03;
  double robot_radius = .18;
  std::vector<Eigen::Vector2d> footprint;
  double regulated_min_speed = .08;
  bool collision_monitor_enabled = true;
  double collision_monitor_stop_distance = .10;
  double collision_monitor_slowdown_distance = .28;
  double collision_monitor_slowdown_ratio = .55;
  double collision_monitor_approach_horizon = .90;
  double collision_monitor_source_timeout = .3;
  int collision_monitor_trigger_cycles = 2;
  int collision_monitor_release_cycles = 2;
  bool collision_monitor_passage_mode = false;
  double control_period = .06;
};

CollisionMonitorParameters CollisionMonitorParametersFrom(const NavigationConfig& config);
}  // namespace navigation2d
