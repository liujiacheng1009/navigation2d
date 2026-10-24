#include "navigation2d/application/navigation_system.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include "navigation2d/application/algorithm_registry.h"
#include "navigation2d/application/command_safety_pipeline.h"
#include "navigation2d/application/docking_servo.h"
#include "navigation2d/application/execution_watchdog.h"
#include "navigation2d/application/navigation_failure.h"
#include "navigation2d/application/route_preparation.h"
#include "navigation2d/control/collision_monitor.h"
#include "navigation2d/costmap/grid_2d.h"
#include "navigation2d/costmap/layered_costmap.h"
#include "navigation2d/planning/collision_checker.h"
#include "navigation2d/planning/global_planner.h"
#include "navigation2d/planning/path_geometry.h"
#include "navigation2d/planning/path_metrics.h"

namespace navigation2d {
using planning_internal::AddPathMetrics;
using planning_internal::DistanceToPath;
using planning_internal::PathFootprintValid;
using planning_internal::PathProgress;
namespace {
double NormalizeAngle(double value) { return std::atan2(std::sin(value), std::cos(value)); }

// Planning clearance is the body circle. `LayeredCostmap::lethal` already
// measures distance to the occupied cell box, so adding another half-cell
// diagonal seals a passage the chassis fits (a door whose free half-width
// sits between kPlanningRadius() and kPlanningRadius() + half-diagonal).
double EffectivePlanningRadius(const NavigationConfig& config, const LayeredCostmap&) {
  return config.robot_radius;
}

}  // namespace

class NavigationSystem::Impl {
 public:
  Impl(NavigationConfig value, const std::string& map_path,
       std::shared_ptr<const AlgorithmRegistry> registry)
      : config(std::move(value)),
        algorithms(std::move(registry)),
        costmap(Grid2d::Load(map_path), config),
        planner(algorithms->CreatePlanner(config)),
        controller(algorithms->CreateController(config)),
        planner_traits(algorithms->PlannerTraits(config.planner)),
        safety(config),
        docking({config.goal_xy_tolerance, config.goal_yaw_tolerance,
                 config.max_linear_acceleration, config.max_angular_acceleration,
                 config.control_period}),
        execution({config.control_period, config.progress_radius, config.progress_timeout,
                   config.max_recovery_attempts}) {
    if (std::abs(costmap.grid().resolution() - config.map_resolution) > 1e-9) {
      throw std::runtime_error("map resolution does not match navigation configuration");
    }
  }

  void Replan(const Pose2d& pose) {
    ++state.replans;
    try {
      const auto planning_started = std::chrono::steady_clock::now();
      auto prepared = PrepareRoute(*planner, costmap, config, planner_traits,
                                   {pose, *goal, banned_path_signatures});
      path = std::move(prepared.path);
      state.path_signature = prepared.signature;
      // Keep Theta* long chords when they stay footprint-valid. Narrow-door
      // entry uses a tighter radius / A*, not a planner swap for a few
      // centimetres of clearance (that made sawtooth GLOBAL).
      // A replan is a new ordered route contract. Reset the controller's
      // internal projection state even when the new path has similar endpoints.
      controller = algorithms->CreateController(config);
      state.global_planning = planner->Diagnostics();
      state.planning_failure_reason.clear();
      const double planning_elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - planning_started)
              .count();
      state.global_planning.elapsed_s = planning_elapsed;
      if (state.global_planning.first_solution_s <= 0.) {
        state.global_planning.first_solution_s = planning_elapsed;
      }
      AddPathMetrics(path, costmap, config.robot_radius, &state.global_planning);
      if (state.global_planning.obstacle_heuristic_cache_hit) {
        ++state.obstacle_heuristic_cache_hits;
      }
      if (state.global_planning.incremental_reuse) {
        ++state.incremental_replans;
      }
      state.incremental_repaired_states += state.global_planning.repaired_states;
      double path_length = 0.;
      for (std::size_t i = 1; i < path.size(); ++i) {
        path_length += (path[i].translation() - path[i - 1].translation()).norm();
      }
      if (state.global_path_length_m == 0.) {
        state.global_path_length_m = path_length;
      }
      state.path_progress_m = 0.;
      state.phase = NavigationPhase::kAlignToPath;
      execution.NewRoute();
      planning_failures = 0;
      state.status = NavigationStatus::kNavigating;
    } catch (const std::runtime_error& error) {
      path.clear();
      state.command = {};
      state.path_signature = 0;
      execution.PlanningFailed();
      state.planning_failure_reason = error.what();
      ++planning_failures;
      state.status = planning_failures > config.max_recovery_attempts
                         ? NavigationStatus::kBlocked
                         : NavigationStatus::kNavigating;
    }
  }

  bool ApplyFailure(NavigationFailure failure) {
    if (failure == NavigationFailure::kNone) {
      return false;
    }
    const auto effects = EffectsFor(failure);
    if (effects.record_recovery) {
      ++state.recoveries;
    }
    if (effects.clear_route) {
      path.clear();
    }
    state.command = {};
    if (effects.blocked) {
      state.status = NavigationStatus::kBlocked;
    }
    if (effects.reset_replan_timer) {
      next_replan_time = 0.;
    }
    if (!effects.message.empty()) {
      state.planning_failure_reason = effects.message;
    }
    return true;
  }

  NavigationConfig config;
  std::shared_ptr<const AlgorithmRegistry> algorithms;
  LayeredCostmap costmap;
  std::unique_ptr<GlobalPlanner> planner;
  std::unique_ptr<LocalController> controller;
  PlannerCapabilities planner_traits;
  CommandSafetyPipeline safety;
  std::optional<Pose2d> goal;
  Path path;
  NavigationState state;
  bool observation_changed = false;
  double next_replan_time = 0.;
  DockingServo docking;
  ExecutionWatchdog execution;
  int planning_failures = 0;
  std::unordered_set<std::uint64_t> banned_path_signatures;
};

NavigationSystem::NavigationSystem(NavigationConfig config, const std::string& map_path)
    : NavigationSystem(std::move(config), map_path, AlgorithmRegistry::Builtins()) {}

NavigationSystem::NavigationSystem(NavigationConfig config, const std::string& map_path,
                                   std::shared_ptr<const AlgorithmRegistry> algorithms) {
  if (!algorithms) {
    throw std::invalid_argument("null algorithm registry");
  }
  algorithms->Validate(config);
  impl_ = std::make_unique<Impl>(std::move(config), map_path, std::move(algorithms));
}
NavigationSystem::~NavigationSystem() = default;
NavigationSystem::NavigationSystem(NavigationSystem&&) noexcept = default;
NavigationSystem& NavigationSystem::operator=(NavigationSystem&&) noexcept = default;

const Path& NavigationSystem::path() const { return impl_->path; }
const std::string& NavigationSystem::planner_id() const { return impl_->config.planner; }
const std::string& NavigationSystem::controller_id() const { return impl_->config.controller; }

void NavigationSystem::SetGoal(Pose2d goal) {
  impl_->goal = goal;
  impl_->path.clear();
  impl_->state.status = NavigationStatus::kNavigating;
  impl_->state.global_path_length_m = 0.;
  impl_->next_replan_time = 0.;
  impl_->execution.NewGoal();
  impl_->state.path_progress_m = 0.;
  impl_->planning_failures = 0;
  impl_->state.phase = NavigationPhase::kAlignToPath;
  impl_->docking.Reset();
  impl_->state.controller_commanded_motion = false;
  impl_->state.safety_stopped_motion = false;
  impl_->state.requested_command = {};
  impl_->state.published_command = {};
  impl_->state.collision_monitor_action = CollisionMonitorAction::kNone;
  impl_->state.planning_failure_reason.clear();
  impl_->state.path_signature = 0;
}

void NavigationSystem::BanPathSignatures(std::vector<std::uint64_t> signatures) {
  impl_->banned_path_signatures.clear();
  impl_->banned_path_signatures.insert(signatures.begin(), signatures.end());
}

void NavigationSystem::ClearGoal() {
  impl_->goal.reset();
  impl_->path.clear();
  impl_->state.command = {};
  impl_->state.status = NavigationStatus::kIdle;
}

void NavigationSystem::UpdateLaserScan(const Pose2d& sensor_pose, const LaserScan& scan) {
  impl_->safety.UpdateLaserScan(sensor_pose, scan);
  const auto before = impl_->costmap.digest();
  impl_->costmap.UpdateObstacleLayer(sensor_pose, scan);
  impl_->observation_changed = impl_->observation_changed || before != impl_->costmap.digest();
}

void NavigationSystem::UpdateCollisionMonitorLaserScan(const Pose2d& sensor_pose,
                                                       const LaserScan& scan) {
  impl_->safety.UpdateLaserScan(sensor_pose, scan);
}

void NavigationSystem::UpdateCollisionMonitorDepthPoints(
    const Pose2d& robot_pose, const std::vector<Eigen::Vector2d>& base_points, double timestamp) {
  impl_->safety.UpdateDepthPoints(robot_pose, base_points, timestamp);
}

void NavigationSystem::UpdatePointCloud(const Pose2d& sensor_pose, const PointCloud2d& cloud) {
  impl_->safety.UpdatePointCloud(sensor_pose, cloud);
  const auto before = impl_->costmap.digest();
  impl_->costmap.UpdateObstacleLayer(sensor_pose, cloud);
  impl_->observation_changed = impl_->observation_changed || before != impl_->costmap.digest();
}

void NavigationSystem::UpdateDynamicObstacles(std::vector<PredictedObstacle> obstacles) {
  impl_->safety.SetPredictions(std::move(obstacles));
}

void NavigationSystem::UpdateGuidanceCandidates(std::vector<GuidanceCandidate> candidates) {
  impl_->controller->SetGuidanceCandidates(std::move(candidates));
}

NavigationState NavigationSystem::ComputeCommand(const Pose2d& pose, Twist2d measured_velocity,
                                                 double timestamp) {
  // Treat odometry as a measured input, not an unlimited control state. Some
  // simulator diff-drive backends emit a one-sample angular derivative across
  // yaw wrapping (observed at >120 rad/s). Feeding that value into acceleration
  // limiting makes every physically valid command unreachable for many cycles.
  // Project feedback onto the robot's declared actuation envelope; pose itself
  // remains the unmodified ground-truth pose supplied by the caller.
  if (!std::isfinite(measured_velocity.linear) ||
      std::abs(measured_velocity.linear) > 2. * impl_->config.desired_linear_velocity) {
    measured_velocity.linear = impl_->state.published_command.linear;
  }
  if (!std::isfinite(measured_velocity.angular) ||
      std::abs(measured_velocity.angular) > 2. * impl_->config.max_angular_velocity) {
    measured_velocity.angular = impl_->state.published_command.angular;
  }
  measured_velocity.linear =
      std::clamp(measured_velocity.linear, -impl_->config.max_reverse_velocity,
                 impl_->config.desired_linear_velocity);
  measured_velocity.angular =
      std::clamp(measured_velocity.angular, -impl_->config.max_angular_velocity,
                 impl_->config.max_angular_velocity);
  impl_->state.controller_solve_us = 0.;
  if (!impl_->goal) {
    impl_->state.command = {};
    impl_->state.status = NavigationStatus::kIdle;
    return impl_->state;
  }
  impl_->execution.BeginCycle(timestamp);
  // A global path is a route contract, not a high-rate control signal.  Do not
  // replan every scan — that makes the controller chase a discontinuous
  // reference. But once the live obstacle layer makes the committed chord
  // footprint-illegal, drop it immediately so we never keep publishing a
  // path that cuts through newly observed walls/specks.
  if (!impl_->path.empty()) {
    const double planning_radius = impl_->planner_traits.owns_smoothing
                                       ? impl_->config.robot_radius
                                       : EffectivePlanningRadius(impl_->config, impl_->costmap);
    if (!PathFootprintValid(impl_->path, impl_->costmap, planning_radius)) {
      impl_->path.clear();
      impl_->state.planning_failure_reason = "active path collided with updated obstacles";
      impl_->state.command = {};
      impl_->next_replan_time = 0.;
    }
  }
  if (impl_->path.empty()) {
    impl_->Replan(pose);
    impl_->observation_changed = false;
    impl_->execution.PlanningAttempted(timestamp);
  }
  // Planning can legitimately fail while an online frontier map is changing.
  // Do not run a stale progress watchdog without a route to measure against.
  if (impl_->path.empty()) {
    return impl_->state;
  }
  const double goal_distance = (impl_->goal->translation() - pose.translation()).norm();
  const bool precision_goal = impl_->config.goal_xy_tolerance <= .05;
  // A centimetre-scale SE(2) docking pose is not a short global path. Switch
  // to a dedicated pose servo before the remaining polyline becomes smaller
  // than RPP's lookahead and its progress coordinate loses useful meaning.
  if (precision_goal && goal_distance <= .30) {
    impl_->state.phase = NavigationPhase::kDockToGoal;
    const auto servo =
        impl_->docking.Advance(pose, *impl_->goal, goal_distance, impl_->state.published_command);
    const double yaw_error = servo.yaw_error_rad;
    impl_->execution.NoteDocking(servo.distance_progress, servo.yaw_progress, timestamp);
    if (servo.succeeded) {
      impl_->state.command = {};
      impl_->state.published_command = {};
      impl_->state.status = NavigationStatus::kSucceeded;
      return impl_->state;
    }
    Twist2d docking = servo.command;
    impl_->state.requested_command = docking;
    impl_->state.controller_commanded_motion =
        std::abs(docking.linear) > 1e-4 || std::abs(docking.angular) > 1e-4;
    const auto monitored = impl_->safety.Filter(pose, docking, timestamp);
    docking = monitored.command;
    impl_->state.published_command = docking;
    impl_->state.collision_monitor_action = monitored.action;
    impl_->state.safety_stopped_motion = impl_->state.controller_commanded_motion &&
                                         std::abs(docking.linear) <= 1e-4 &&
                                         std::abs(docking.angular) <= 1e-4;
    impl_->state.minimum_ttc_s = monitored.time_to_collision_s;
    const bool docking_commanded =
        impl_->state.controller_commanded_motion &&
        (std::abs(docking.linear) > 1e-4 || std::abs(docking.angular) > 1e-4) &&
        !impl_->state.safety_stopped_motion;
    if (impl_->ApplyFailure(
            impl_->execution.ObserveStall({pose, 0., measured_velocity, yaw_error, timestamp,
                                           docking_commanded, std::abs(docking.linear) > 1e-3},
                                          StallMode::kDocking))) {
      return impl_->state;
    }
    if (impl_->ApplyFailure(impl_->execution.DockingTimeout(timestamp))) {
      return impl_->state;
    }
    impl_->state.command = docking;
    impl_->state.status = NavigationStatus::kNavigating;
    return impl_->state;
  }
  if (goal_distance <= impl_->config.goal_xy_tolerance) {
    const double yaw_error = NormalizeAngle(Yaw(*impl_->goal) - Yaw(pose));
    if (std::abs(yaw_error) <= impl_->config.goal_yaw_tolerance) {
      impl_->state.command = {};
      impl_->state.status = NavigationStatus::kSucceeded;
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
  // Translating in a circle is not navigation progress.  Measure progress in
  // the path's arc-length coordinate instead of straight-line distance to the
  // goal: a correct path around an obstacle can temporarily move farther from
  // the goal, but it never moves backward along its committed route.
  const double path_progress = PathProgress(impl_->path, pose, impl_->execution.path_anchor_m());
  impl_->state.path_progress_m = path_progress;
  if (impl_->state.phase == NavigationPhase::kAlignToPath && impl_->path.size() >= 2) {
    const auto tangent = impl_->path[1].translation() - impl_->path[0].translation();
    const double heading_error = NormalizeAngle(std::atan2(tangent.y(), tangent.x()) - Yaw(pose));
    const double alignment_score = std::abs(heading_error) + DistanceToPath(impl_->path, pose);
    if (impl_->execution.ObserveAlignment(alignment_score, heading_error, path_progress,
                                          timestamp)) {
      impl_->state.phase = NavigationPhase::kTrackPath;
    } else {
      // Align is a rotation-only contract.  RPP's first tracking sample from
      // rest is accel-limited to about (0.05, 0.11); CommandStalled then
      // treats that as a failed translation and ignores yaw, so a valid
      // 17 m home path is torn down before the base has finished turning.
      const double dv = impl_->config.max_linear_acceleration * impl_->config.control_period;
      const double dw = impl_->config.max_angular_acceleration * impl_->config.control_period;
      Twist2d align{0., std::clamp(2.2 * heading_error, -impl_->config.max_angular_velocity,
                                   impl_->config.max_angular_velocity)};
      align.linear = std::clamp(0., impl_->state.published_command.linear - dv,
                                impl_->state.published_command.linear + dv);
      align.angular = std::clamp(align.angular, impl_->state.published_command.angular - dw,
                                 impl_->state.published_command.angular + dw);
      impl_->state.requested_command = align;
      impl_->state.controller_commanded_motion =
          std::abs(align.linear) > 1e-4 || std::abs(align.angular) > 1e-4;
      impl_->execution.ChargeRotation(align.angular);
      const auto monitored = impl_->safety.Filter(pose, align, timestamp);
      align = monitored.command;
      impl_->state.published_command = align;
      impl_->state.collision_monitor_action = monitored.action;
      impl_->state.safety_stopped_motion = impl_->state.controller_commanded_motion &&
                                           std::abs(align.linear) <= 1e-4 &&
                                           std::abs(align.angular) <= 1e-4;
      impl_->state.minimum_ttc_s = monitored.time_to_collision_s;
      impl_->state.controller_diagnostics.backend = ControllerBackend::kRpp;
      impl_->state.controller_diagnostics.maneuver = ControllerManeuver::kRotateToPath;
      impl_->state.controller_diagnostics.fallback_level = 6;
      const bool align_commanded =
          impl_->state.controller_commanded_motion &&
          (std::abs(align.linear) > 1e-4 || std::abs(align.angular) > 1e-4) &&
          !impl_->state.safety_stopped_motion;
      if (impl_->ApplyFailure(impl_->execution.ObserveStall(
              {pose, path_progress, measured_velocity, heading_error, timestamp, align_commanded,
               std::abs(align.linear) > 1e-3},
              StallMode::kAlignment))) {
        return impl_->state;
      }
      if (impl_->ApplyFailure(impl_->execution.CheckRotation(RotationStage::kAlignment))) {
        return impl_->state;
      }
      impl_->state.command = align;
      impl_->state.status = NavigationStatus::kNavigating;
      impl_->state.costmap_digest = impl_->costmap.digest();
      return impl_->state;
    }
  }
  if (impl_->ApplyFailure(
          impl_->execution.ObserveSafetyStop(impl_->state.safety_stopped_motion, timestamp))) {
    return impl_->state;
  }
  if (impl_->ApplyFailure(impl_->execution.ObservePrevious(
          {impl_->state.command, path_progress, timestamp, impl_->state.safety_stopped_motion,
           impl_->state.phase == NavigationPhase::kAlignToPath}))) {
    return impl_->state;
  }

  Twist2d command;
  {
    double route_heading_error = 0.;
    if (impl_->path.size() >= 2) {
      const auto tangent = impl_->path[1].translation() - impl_->path[0].translation();
      route_heading_error = NormalizeAngle(std::atan2(tangent.y(), tangent.x()) - Yaw(pose));
    }
    const auto solve_started = std::chrono::steady_clock::now();
    command = impl_->controller->Compute(impl_->path, pose, measured_velocity, impl_->costmap,
                                         impl_->safety.predictions());
    impl_->state.requested_command = command;
    impl_->state.controller_commanded_motion =
        std::abs(command.linear) > 1e-4 || std::abs(command.angular) > 1e-4;
    impl_->execution.ChargeController(command);
    impl_->state.controller_solve_us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - solve_started)
            .count();
    impl_->state.controller_diagnostics = impl_->controller->Diagnostics();
    if (impl_->state.controller_diagnostics.solve_us <= 0.) {
      impl_->state.controller_diagnostics.solve_us = impl_->state.controller_solve_us;
    }
    const auto monitored = impl_->safety.Filter(pose, command, timestamp);
    command = monitored.command;
    impl_->state.published_command = command;
    impl_->state.collision_monitor_action = monitored.action;
    impl_->state.safety_stopped_motion = impl_->state.controller_commanded_motion &&
                                         std::abs(command.linear) <= 1e-4 &&
                                         std::abs(command.angular) <= 1e-4;
    const bool command_reached_base =
        impl_->state.controller_commanded_motion &&
        (std::abs(command.linear) > 1e-4 || std::abs(command.angular) > 1e-4) &&
        !impl_->state.safety_stopped_motion &&
        !impl_->state.controller_diagnostics.intentional_stop;
    if (impl_->ApplyFailure(impl_->execution.ObserveStall(
            {pose, path_progress, measured_velocity, route_heading_error, timestamp,
             command_reached_base, std::abs(command.linear) > 1e-3},
            StallMode::kTracking))) {
      return impl_->state;
    }
    if (impl_->ApplyFailure(impl_->execution.ObserveController(
            impl_->state.controller_commanded_motion, impl_->state.safety_stopped_motion,
            impl_->state.controller_diagnostics.intentional_stop))) {
      return impl_->state;
    }
    if (impl_->ApplyFailure(impl_->execution.CheckRotation(RotationStage::kAfterController))) {
      return impl_->state;
    }
    const double dynamic_ttc = impl_->safety.DynamicTtc(pose, command);
    impl_->state.minimum_ttc_s = monitored.time_to_collision_s > 0. && dynamic_ttc > 0.
                                     ? std::min(monitored.time_to_collision_s, dynamic_ttc)
                                     : std::max(monitored.time_to_collision_s, dynamic_ttc);
  }
  if (command.linear == 0. && command.angular == 0. &&
      (measured_velocity.linear != 0. || measured_velocity.angular != 0.)) {
    ++impl_->state.emergency_stops;
  }
  impl_->state.command = command;
  impl_->state.status = NavigationStatus::kNavigating;
  impl_->state.costmap_digest = impl_->costmap.digest();
  return impl_->state;
}

}  // namespace navigation2d
