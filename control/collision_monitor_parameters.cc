#include <utility>

#include "navigation2d/control/collision_monitor.h"

namespace navigation2d {
namespace {
template <class Config>
CollisionMonitorParameters Project(Config&& config) {
  return {.map_resolution = config.map_resolution,
          .robot_radius = config.robot_radius,
          .footprint = std::forward<Config>(config).footprint,
          .regulated_min_speed = config.regulated_min_speed,
          .collision_monitor_enabled = config.collision_monitor_enabled,
          .collision_monitor_stop_distance = config.collision_monitor_stop_distance,
          .collision_monitor_slowdown_distance = config.collision_monitor_slowdown_distance,
          .collision_monitor_slowdown_ratio = config.collision_monitor_slowdown_ratio,
          .collision_monitor_approach_horizon = config.collision_monitor_approach_horizon,
          .collision_monitor_source_timeout = config.collision_monitor_source_timeout,
          .collision_monitor_trigger_cycles = config.collision_monitor_trigger_cycles,
          .collision_monitor_release_cycles = config.collision_monitor_release_cycles,
          .collision_monitor_passage_mode = config.collision_monitor_passage_mode,
          .control_period = config.control_period};
}
}  // namespace

CollisionMonitorParameters CollisionMonitorParametersFrom(const NavigationConfig& config) {
  return Project(config);
}

CollisionMonitor::CollisionMonitor(NavigationConfig config)
    : CollisionMonitor(Project(std::move(config)), ParametersTag{}) {}

CollisionMonitor::CollisionMonitor(CollisionMonitorParameters parameters, ParametersTag)
    : config_(std::move(parameters)) {}

CollisionMonitor CollisionMonitor::WithParameters(CollisionMonitorParameters parameters) {
  return CollisionMonitor(std::move(parameters), ParametersTag{});
}
}  // namespace navigation2d
