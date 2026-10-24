#include "navigation2d/configuration/navigation_yaml.h"
#include "navigation2d/configuration/yaml_schema.h"
namespace navigation2d::configuration_detail {
void ReadSafetyYaml(const YAML::Node& root, NavigationConfig& c) {
  const auto safety = root["safety"];
  ExactKeys(safety,
            {"collision_horizon", "max_navigation_duration", "goal_xy_tolerance",
             "goal_yaw_tolerance", "collision_monitor_enabled", "collision_monitor_stop_distance",
             "collision_monitor_slowdown_distance", "collision_monitor_slowdown_ratio",
             "collision_monitor_approach_horizon", "collision_monitor_source_timeout",
             "collision_monitor_trigger_cycles", "collision_monitor_release_cycles"},
            "safety");
  c.collision_horizon = Number(safety, "collision_horizon");
  c.max_navigation_duration = Number(safety, "max_navigation_duration");
  c.goal_xy_tolerance = Number(safety, "goal_xy_tolerance");
  c.goal_yaw_tolerance = Number(safety, "goal_yaw_tolerance");
  c.collision_monitor_enabled = safety["collision_monitor_enabled"].as<bool>();
  c.collision_monitor_stop_distance = Number(safety, "collision_monitor_stop_distance");
  c.collision_monitor_slowdown_distance = Number(safety, "collision_monitor_slowdown_distance");
  c.collision_monitor_slowdown_ratio = Number(safety, "collision_monitor_slowdown_ratio");
  c.collision_monitor_approach_horizon = Number(safety, "collision_monitor_approach_horizon");
  c.collision_monitor_source_timeout = Number(safety, "collision_monitor_source_timeout");
  c.collision_monitor_trigger_cycles = safety["collision_monitor_trigger_cycles"].as<int>();
  c.collision_monitor_release_cycles = safety["collision_monitor_release_cycles"].as<int>();
  const auto scheduler = root["scheduler"];
  ExactKeys(scheduler, {"global_replan_period"}, "scheduler");
  c.global_replan_period = Number(scheduler, "global_replan_period");
  const auto recovery = root["recovery"];
  ExactKeys(recovery,
            {"progress_radius", "progress_timeout", "duration", "linear_velocity",
             "angular_velocity", "dynamic_obstacle_radius"},
            "recovery");
  c.progress_radius = Number(recovery, "progress_radius");
  c.progress_timeout = Number(recovery, "progress_timeout");
  c.recovery_duration = Number(recovery, "duration");
  c.recovery_linear_velocity = Number(recovery, "linear_velocity");
  c.recovery_angular_velocity = Number(recovery, "angular_velocity");
  c.dynamic_obstacle_radius = Number(recovery, "dynamic_obstacle_radius");
}
}  // namespace navigation2d::configuration_detail
