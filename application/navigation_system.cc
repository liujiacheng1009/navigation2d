#include "navigation2d/application/navigation_system.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <limits>
#include <stdexcept>
#include <utility>
#include "navigation2d/control/regulated_pure_pursuit.h"
#include "navigation2d/control/dwa_controller.h"
#include "navigation2d/control/mppi_controller.h"
#include "navigation2d/control/mpc_controller.h"
#include "navigation2d/control/dynamic_safety.h"
#include "navigation2d/costmap/grid_2d.h"
#include "navigation2d/costmap/layered_costmap.h"
#include "navigation2d/planning/global_planner.h"
#include "navigation2d/planning/planner_factory.h"
#include "navigation2d/planning/collision_checker.h"

namespace navigation2d {
namespace {
double NormalizeAngle(double value) { return std::atan2(std::sin(value), std::cos(value)); }
void AddPathMetrics(const Path& path, const LayeredCostmap& costmap, double robot_radius,
                    GlobalPlanningDiagnostics* diagnostics) {
  planning_internal::DistanceField field(costmap);
  diagnostics->path_min_clearance_m = std::numeric_limits<double>::infinity();
  diagnostics->path_max_curvature = 0.;
  for (const auto& pose : path)
    diagnostics->path_min_clearance_m = std::min(
        diagnostics->path_min_clearance_m,
        std::max(0., field.distance(X(pose), Y(pose)) - robot_radius));
  for (std::size_t index = 1; index + 1 < path.size(); ++index) {
    const auto first = path[index].translation() - path[index - 1].translation();
    const auto second = path[index + 1].translation() - path[index].translation();
    const double a = first.norm(), b = second.norm();
    const double chord = (path[index + 1].translation() - path[index - 1].translation()).norm();
    if (a > 1e-8 && b > 1e-8 && chord > 1e-8)
      diagnostics->path_max_curvature = std::max(diagnostics->path_max_curvature,
          2. * std::abs(first.x() * second.y() - first.y() * second.x()) / (a * b * chord));
  }
}
std::unique_ptr<LocalController> MakeController(const NavigationConfig& config) {
  if (config.controller == "rpp") return std::make_unique<RegulatedPurePursuit>(config);
  if (config.controller == "dwa") return std::make_unique<DwaController>(config);
  if (config.controller == "mppi") return std::make_unique<MppiController>(config);
  if (config.controller == "mpc") return std::make_unique<MpcController>(config);
  throw std::runtime_error("unknown controller: " + config.controller);
}
}

class NavigationSystem::Impl {
 public:
  Impl(NavigationConfig value, const std::string& map_path)
      : config(std::move(value)), costmap(Grid2d::Load(map_path), config),
        planner(MakeGlobalPlanner(config)), controller(MakeController(config)) {
    if (std::abs(costmap.grid().resolution() - config.map_resolution) > 1e-9)
      throw std::runtime_error("map resolution does not match navigation configuration");
  }

  void Replan(const Pose2d& pose) {
    try {
      const auto planning_started = std::chrono::steady_clock::now();
      path = planner->Plan(costmap, pose, *goal); ++state.replans;
      state.global_planning = planner->Diagnostics();
      const double planning_elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - planning_started).count();
      state.global_planning.elapsed_s = planning_elapsed;
      if (state.global_planning.first_solution_s <= 0.)
        state.global_planning.first_solution_s = planning_elapsed;
      AddPathMetrics(path, costmap, config.robot_radius, &state.global_planning);
      if (state.global_planning.obstacle_heuristic_cache_hit)
        ++state.obstacle_heuristic_cache_hits;
      if (state.global_planning.incremental_reuse) ++state.incremental_replans;
      state.incremental_repaired_states += state.global_planning.repaired_states;
      double path_length = 0.;
      for (std::size_t i = 1; i < path.size(); ++i)
        path_length += (path[i].translation() - path[i - 1].translation()).norm();
      if (state.global_path_length_m == 0.) state.global_path_length_m = path_length;
      state.status = NavigationStatus::kNavigating;
    } catch (const std::runtime_error&) {
      path.clear(); state.command = {}; state.status = NavigationStatus::kBlocked;
      ++state.emergency_stops;
    }
  }

  NavigationConfig config;
  LayeredCostmap costmap;
  std::unique_ptr<GlobalPlanner> planner;
  std::unique_ptr<LocalController> controller;
  std::optional<Pose2d> goal;
  Path path;
  NavigationState state;
  bool observation_changed = false;
  double next_replan_time = 0.;
  double last_progress_time = 0.;
  Pose2d last_progress_pose;
  bool progress_initialized = false;
  int recovery_steps_remaining = 0;
  std::vector<PredictedObstacle> dynamic_obstacles;
};

NavigationSystem::NavigationSystem(NavigationConfig config, const std::string& map_path)
    : impl_(std::make_unique<Impl>(std::move(config), map_path)) {}
NavigationSystem::~NavigationSystem() = default;
NavigationSystem::NavigationSystem(NavigationSystem&&) noexcept = default;
NavigationSystem& NavigationSystem::operator=(NavigationSystem&&) noexcept = default;

void NavigationSystem::SetGoal(Pose2d goal) {
  impl_->goal = goal; impl_->path.clear(); impl_->state.status = NavigationStatus::kNavigating;
  impl_->state.global_path_length_m = 0.;
  impl_->next_replan_time = 0.; impl_->progress_initialized = false;
}

void NavigationSystem::ClearGoal() {
  impl_->goal.reset(); impl_->path.clear(); impl_->state.command = {};
  impl_->state.status = NavigationStatus::kIdle;
}

void NavigationSystem::UpdateLaserScan(const Pose2d& sensor_pose, const LaserScan& scan) {
  const auto before = impl_->costmap.digest();
  impl_->costmap.UpdateObstacleLayer(sensor_pose, scan);
  impl_->observation_changed = impl_->observation_changed || before != impl_->costmap.digest();
}

void NavigationSystem::UpdatePointCloud(const Pose2d& sensor_pose, const PointCloud2d& cloud) {
  const auto before = impl_->costmap.digest();
  impl_->costmap.UpdateObstacleLayer(sensor_pose, cloud);
  impl_->observation_changed = impl_->observation_changed || before != impl_->costmap.digest();
}

void NavigationSystem::UpdateDynamicObstacles(std::vector<PredictedObstacle> obstacles) {
  impl_->dynamic_obstacles = std::move(obstacles);
}

NavigationState NavigationSystem::ComputeCommand(const Pose2d& pose, Twist2d measured_velocity,
                                                 double timestamp) {
  impl_->state.controller_solve_us = 0.;
  if (!impl_->goal) { impl_->state.command = {}; impl_->state.status = NavigationStatus::kIdle; return impl_->state; }
  if (!impl_->progress_initialized) {
    impl_->last_progress_pose = pose; impl_->last_progress_time = timestamp;
    impl_->progress_initialized = true;
  }
  if (impl_->observation_changed || impl_->path.empty() || timestamp >= impl_->next_replan_time) {
    impl_->Replan(pose); impl_->observation_changed = false;
    impl_->next_replan_time = timestamp + impl_->config.global_replan_period;
  }
  const double goal_distance = (impl_->goal->translation() - pose.translation()).norm();
  if (goal_distance <= impl_->config.goal_xy_tolerance) {
    const double yaw_error = NormalizeAngle(Yaw(*impl_->goal) - Yaw(pose));
    if (std::abs(yaw_error) <= impl_->config.goal_yaw_tolerance) {
      impl_->state.command = {}; impl_->state.status = NavigationStatus::kSucceeded;
      return impl_->state;
    }
    const double target = std::clamp(2. * yaw_error, -impl_->config.max_angular_velocity,
                                    impl_->config.max_angular_velocity);
    const double max_delta = impl_->config.max_angular_acceleration * impl_->config.control_period;
    impl_->state.command = {0., std::clamp(target, measured_velocity.angular - max_delta,
                                          measured_velocity.angular + max_delta)};
    impl_->state.status = NavigationStatus::kNavigating;
    return impl_->state;
  }
  if ((pose.translation() - impl_->last_progress_pose.translation()).norm() >=
      impl_->config.progress_radius) {
    impl_->last_progress_pose = pose; impl_->last_progress_time = timestamp;
  } else if (timestamp - impl_->last_progress_time > impl_->config.progress_timeout) {
    ++impl_->state.recoveries; impl_->last_progress_time = timestamp;
    impl_->recovery_steps_remaining = static_cast<int>(
        std::ceil(impl_->config.recovery_duration / impl_->config.control_period));
  }
  if (impl_->path.empty() && impl_->recovery_steps_remaining == 0) return impl_->state;

  Twist2d command;
  if (impl_->recovery_steps_remaining > 0) {
    command = {std::max(-impl_->config.max_reverse_velocity,
                        impl_->config.recovery_linear_velocity), 0.};
    if (impl_->controller->CollisionImminent(pose, command, impl_->costmap))
      command = {0., impl_->config.recovery_angular_velocity};
    --impl_->recovery_steps_remaining;
  } else {
    const auto solve_started = std::chrono::steady_clock::now();
    command = impl_->controller->Compute(impl_->path, pose, measured_velocity, impl_->costmap,
                                         impl_->dynamic_obstacles);
    impl_->state.controller_solve_us = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - solve_started).count();
    if (DynamicCollisionImminent(pose, command, impl_->dynamic_obstacles, impl_->config))
      command = {};
  }
  if (command.linear == 0. && command.angular == 0. &&
      (measured_velocity.linear != 0. || measured_velocity.angular != 0.))
    ++impl_->state.emergency_stops;
  impl_->state.command = command;
  impl_->state.status = NavigationStatus::kNavigating;
  impl_->state.costmap_digest = impl_->costmap.digest();
  return impl_->state;
}

}  // namespace navigation2d
