#include "navigation2d/simulation/navigation_simulator.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <chrono>
#include <limits>
#include "navigation2d/application/navigation_system.h"
#include "navigation2d/costmap/grid_2d.h"
#include "navigation2d/planning/collision_checker.h"

namespace navigation2d::simulation {
namespace {
double Angle(double value) { return std::atan2(std::sin(value), std::cos(value)); }

LaserScan SimulateScan(const Pose2d& pose, const std::vector<ObstacleEvent>& obstacles,
                       const std::vector<bool>& active, double now, const NavigationConfig& config) {
  LaserScan scan{-M_PI, 2. * M_PI / 360., .05, config.raytrace_max_range,
                 std::vector<double>(360, config.raytrace_max_range)};
  for (std::size_t ray = 0; ray < scan.ranges.size(); ++ray) {
    const double angle = Yaw(pose) + scan.angle_min + ray * scan.angle_increment;
    const double ux = std::cos(angle), uy = std::sin(angle);
    for (std::size_t i = 0; i < obstacles.size(); ++i) if (active[i]) {
      const double age = now - obstacles[i].appear_s;
      const double dx = obstacles[i].x + obstacles[i].vx * age - X(pose);
      const double dy = obstacles[i].y + obstacles[i].vy * age - Y(pose);
      const double projection = dx * ux + dy * uy;
      const double perpendicular2 = dx * dx + dy * dy - projection * projection;
      const double radius = config.dynamic_obstacle_radius;
      if (projection > 0. && perpendicular2 <= radius * radius) {
        const double range = projection - std::sqrt(radius * radius - perpendicular2);
        if (range >= scan.range_min) scan.ranges[ray] = std::min(scan.ranges[ray], range);
      }
    }
  }
  return scan;
}
}

RunResult NavigationSimulator::Run(const std::string& world_path, Pose2d pose, Pose2d goal,
                                   const std::vector<ObstacleEvent>& obstacles) const {
  const Grid2d truth = Grid2d::Load(world_path);
  LayeredCostmap truth_costmap(Grid2d::Load(world_path), config_);
  std::optional<planning_internal::FootprintLookup> truth_footprint;
  if (!config_.footprint.empty())
    truth_footprint.emplace(config_.footprint, config_.lattice_yaw_bins,
                            truth.resolution());
  NavigationSystem navigation(config_, world_path);
  navigation.SetGoal(goal);
  RunResult result; result.trajectory.push_back(pose);
  Twist2d velocity;
  Twist2d previous_acceleration;
  std::vector<bool> active(obstacles.size(), false);
  const int max_steps = static_cast<int>(std::ceil(config_.max_navigation_duration /
                                                   config_.control_period));
  int observed_replans = 0;
  for (int step = 0; step < max_steps; ++step) {
    const double now = step * config_.control_period;
    for (std::size_t i = 0; i < obstacles.size(); ++i)
      active[i] = now >= obstacles[i].appear_s &&
          (obstacles[i].disappear_s < 0. || now < obstacles[i].disappear_s);
    // The simulator is the tracker stand-in: it exports the same prediction
    // contract that a production tracker supplies, while the runtime core
    // itself never depends on simulation truth.
    std::vector<PredictedObstacle> predictions;
    for (std::size_t i = 0; i < obstacles.size(); ++i) if (active[i]) {
      const double age = now - obstacles[i].appear_s;
      predictions.push_back({obstacles[i].x + obstacles[i].vx * age,
                             obstacles[i].y + obstacles[i].vy * age,
                             obstacles[i].vx, obstacles[i].vy,
                             config_.dynamic_obstacle_radius, .02, .02});
    }
    navigation.UpdateDynamicObstacles(std::move(predictions));
    if (step % 2 == 0)
      navigation.UpdateLaserScan(pose, SimulateScan(pose, obstacles, active, now, config_));
    const auto cycle_started = std::chrono::steady_clock::now();
    const NavigationState state = navigation.ComputeCommand(pose, velocity, now);
    result.control_cycle_samples_us.push_back(std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - cycle_started).count());
    if (state.replans > observed_replans) {
      observed_replans = state.replans;
      result.global_plan_samples_s.push_back(state.global_planning.elapsed_s);
      result.global_plan_first_solution_samples_s.push_back(
          state.global_planning.first_solution_s);
      result.global_plan_expansions_total += state.global_planning.expansions;
      result.path_min_clearance_m = state.global_planning.path_min_clearance_m;
      result.path_max_curvature = state.global_planning.path_max_curvature;
    }
    if (state.controller_solve_us > 0.)
      result.controller_solve_samples_us.push_back(state.controller_solve_us);
    if (state.controller_diagnostics.solve_us > 0.) {
      result.pure_solver_samples_us.push_back(state.controller_diagnostics.solver_us > 0.
          ? state.controller_diagnostics.solver_us : state.controller_diagnostics.solve_us);
      ++result.controller_commands;
      if (state.controller_diagnostics.deadline_miss) ++result.controller_deadline_misses;
      if (state.controller_diagnostics.fallback_level > 0) ++result.controller_fallbacks;
      if (state.controller_diagnostics.backend == ControllerBackend::kAcados) ++result.acados_commands;
      else if (state.controller_diagnostics.backend == ControllerBackend::kMppi) ++result.mppi_commands;
      else if (state.controller_diagnostics.backend == ControllerBackend::kRpp) ++result.rpp_commands;
    }
    if (state.collision_monitor_action != CollisionMonitorAction::kNone)
      ++result.collision_monitor_interventions;
    if (state.minimum_ttc_s > 0.)
      result.minimum_ttc_s = result.minimum_ttc_s == 0. ? state.minimum_ttc_s :
          std::min(result.minimum_ttc_s, state.minimum_ttc_s);
    if (state.status == NavigationStatus::kSucceeded) {
      result.status = "SUCCEEDED"; result.replans = state.replans;
      result.emergency_stops = state.emergency_stops; result.recoveries = state.recoveries;
      result.global_path_length_m = state.global_path_length_m;
      result.global_planning = state.global_planning;
      result.obstacle_heuristic_cache_hits = state.obstacle_heuristic_cache_hits;
      result.incremental_replans = state.incremental_replans;
      result.incremental_repaired_states = state.incremental_repaired_states;
      result.costmap_digest = state.costmap_digest; break;
    }
    const Twist2d command = state.command;
    const Twist2d acceleration{
        (command.linear - velocity.linear) / config_.control_period,
        (command.angular - velocity.angular) / config_.control_period};
    if (step > 0) {
      result.linear_jerk_samples.push_back(
          std::abs(acceleration.linear - previous_acceleration.linear) / config_.control_period);
      result.angular_jerk_samples.push_back(
          std::abs(acceleration.angular - previous_acceleration.angular) / config_.control_period);
    }
    previous_acceleration = acceleration;
    const double next_yaw = Yaw(pose) + command.angular * config_.control_period;
    Pose2d candidate;
    if (std::abs(command.angular) < 1e-9) {
      candidate = MakePose2d(X(pose) + command.linear * std::cos(Yaw(pose)) * config_.control_period,
                             Y(pose) + command.linear * std::sin(Yaw(pose)) * config_.control_period,
                             Angle(next_yaw));
    } else {
      const double radius = command.linear / command.angular;
      candidate = MakePose2d(X(pose) + radius * (std::sin(next_yaw) - std::sin(Yaw(pose))),
                             Y(pose) - radius * (std::cos(next_yaw) - std::cos(Yaw(pose))),
                             Angle(next_yaw));
    }
    bool collision = truth_footprint
        ? !truth_footprint->CollisionFree(truth_costmap, candidate)
        : truth.collides(X(candidate), Y(candidate), config_.robot_radius);
    for (std::size_t i = 0; i < obstacles.size(); ++i) if (active[i]) {
      const double age = now - obstacles[i].appear_s;
      collision = collision || std::hypot(X(candidate) - (obstacles[i].x + obstacles[i].vx * age),
          Y(candidate) - (obstacles[i].y + obstacles[i].vy * age)) <=
          config_.robot_radius + config_.dynamic_obstacle_radius;
    }
    if (collision) { ++result.collisions; velocity = {}; }
    else { pose = candidate; velocity = command; }
    result.replans = state.replans; result.emergency_stops = state.emergency_stops;
    result.recoveries = state.recoveries; result.costmap_digest = state.costmap_digest;
    result.global_path_length_m = state.global_path_length_m;
    result.global_planning = state.global_planning;
    result.obstacle_heuristic_cache_hits = state.obstacle_heuristic_cache_hits;
    result.incremental_replans = state.incremental_replans;
    result.incremental_repaired_states = state.incremental_repaired_states;
    result.trajectory.push_back(pose); result.steps = step + 1;
  }
  result.duration_s = result.steps * config_.control_period;
  result.goal_error_m = (goal.translation() - pose.translation()).norm();
  result.goal_heading_error_rad = std::abs(Angle(Yaw(goal) - Yaw(pose)));
  return result;
}

}  // namespace navigation2d::simulation
