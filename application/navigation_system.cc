#include "navigation2d/application/navigation_system.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <limits>
#include <stdexcept>
#include <utility>
#include "navigation2d/control/regulated_pure_pursuit.h"
#include "navigation2d/control/collision_monitor.h"
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
Path DensifyPath(const Path& sparse, double maximum_step) {
  if (sparse.size() < 2) return sparse;
  Path dense;
  dense.reserve(sparse.size());
  dense.push_back(sparse.front());
  for (std::size_t index = 1; index < sparse.size(); ++index) {
    const auto start = sparse[index - 1].translation();
    const auto segment = sparse[index].translation() - start;
    const double length = segment.norm();
    if (length <= 1e-9) continue;
    const int steps = std::max(1, static_cast<int>(std::ceil(length / maximum_step)));
    const double yaw = std::atan2(segment.y(), segment.x());
    for (int step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / steps;
      const auto point = start + ratio * segment;
      dense.push_back(MakePose2d(point.x(), point.y(), yaw));
    }
  }
  if (!dense.empty()) dense.back() = MakePose2d(X(sparse.back()), Y(sparse.back()), Yaw(sparse.back()));
  return dense;
}

bool PathFootprintValid(const Path& path, const LayeredCostmap& costmap, double robot_radius) {
  return !path.empty() && std::all_of(path.begin(), path.end(), [&](const Pose2d& pose) {
    return !costmap.lethal(X(pose), Y(pose), robot_radius);
  });
}
// Project onto the *ordered* route, never onto an unconstrained cloud of
// waypoints.  The returned coordinate is arc length from the path start.
//
// The small forward window is intentional: a path may pass close to itself
// in a warehouse aisle.  A global closest-point query can then teleport the
// progress state across a different branch and incorrectly declare motion.
double PathProgress(const Path& path, const Pose2d& pose, double minimum_progress) {
  if (path.size() < 2) return 0.;
  constexpr double kBackwardSlack = .15;
  constexpr double kForwardWindow = 1.25;
  double arc = 0., best_arc = std::max(0., minimum_progress);
  double best_distance = std::numeric_limits<double>::infinity();
  const double begin = std::max(0., minimum_progress - kBackwardSlack);
  const double end = minimum_progress + kForwardWindow;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto& first = path[i - 1].translation();
    const auto& second = path[i].translation();
    const auto segment = second - first;
    const double length = segment.norm();
    if (length <= 1e-9) continue;
    const double segment_end = arc + length;
    if (segment_end < begin) { arc = segment_end; continue; }
    if (arc > end) break;
    const double t = std::clamp((pose.translation() - first).dot(segment) /
                                    (length * length), 0., 1.);
    const double projected_arc = arc + t * length;
    if (projected_arc >= begin && projected_arc <= end) {
      const double distance = (pose.translation() - (first + t * segment)).norm();
      if (distance < best_distance) {
        best_distance = distance;
        best_arc = projected_arc;
      }
    }
    arc = segment_end;
  }
  return best_arc;
}
double DistanceToPath(const Path& path, const Pose2d& pose) {
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1; index < path.size(); ++index) {
    const auto start = path[index - 1].translation();
    const auto segment = path[index].translation() - start;
    const double squared = segment.squaredNorm();
    if (squared <= 1e-12) continue;
    const double ratio = std::clamp((pose.translation() - start).dot(segment) / squared, 0., 1.);
    best = std::min(best, (pose.translation() - (start + ratio * segment)).norm());
  }
  return std::isfinite(best) ? best : 0.;
}
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
        planner(MakeGlobalPlanner(config)), controller(MakeController(config)),
        collision_monitor(config) {
    if (std::abs(costmap.grid().resolution() - config.map_resolution) > 1e-9)
      throw std::runtime_error("map resolution does not match navigation configuration");
  }

  void Replan(const Pose2d& pose) {
    ++state.replans;
    try {
      const auto planning_started = std::chrono::steady_clock::now();
      path = DensifyPath(planner->Plan(costmap, pose, *goal), .08);
      if (!PathFootprintValid(path, costmap, config.robot_radius))
        throw std::runtime_error("planned path failed dense footprint validation");
      // A replan is a new ordered route contract. Reset the controller's
      // internal projection state even when the new path has similar endpoints.
      controller = MakeController(config);
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
      best_path_progress_m = 0.;
      state.path_progress_m = 0.;
      state.phase = NavigationPhase::kAlignToPath;
      best_alignment_score = std::numeric_limits<double>::infinity();
      controller_blocked_cycles = 0;
      rotation_without_progress_rad = 0.;
      planning_failures = 0;
      state.status = NavigationStatus::kNavigating;
    } catch (const std::runtime_error&) {
      path.clear(); state.command = {};
      ++planning_failures;
      state.status = planning_failures > config.max_recovery_attempts ?
          NavigationStatus::kBlocked : NavigationStatus::kNavigating;
    }
  }

  NavigationConfig config;
  LayeredCostmap costmap;
  std::unique_ptr<GlobalPlanner> planner;
  std::unique_ptr<LocalController> controller;
  CollisionMonitor collision_monitor;
  std::optional<Pose2d> goal;
  Path path;
  NavigationState state;
  bool observation_changed = false;
  double next_replan_time = 0.;
  double last_progress_time = 0.;
  // Progress belongs to the route coordinate, not to the Euclidean distance
  // of the goal.  The latter regresses on every legitimate detour around a
  // shelf and was the source of unnecessary turn-in-place recovery cycles.
  double best_path_progress_m = 0.;
  bool progress_initialized = false;
  int recovery_attempts = 0;
  int controller_blocked_cycles = 0;
  int planning_failures = 0;
  double rotation_without_progress_rad = 0.;
  double best_alignment_score = std::numeric_limits<double>::infinity();
  double best_docking_distance = std::numeric_limits<double>::infinity();
  double best_docking_yaw_error = std::numeric_limits<double>::infinity();
  bool docking_position_reached = false;
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
  impl_->best_path_progress_m = 0.;
  impl_->state.path_progress_m = 0.;
  impl_->recovery_attempts = 0; impl_->controller_blocked_cycles = 0;
  impl_->planning_failures = 0;
  impl_->rotation_without_progress_rad = 0.;
  impl_->state.phase = NavigationPhase::kAlignToPath;
  impl_->best_alignment_score = std::numeric_limits<double>::infinity();
  impl_->best_docking_distance = std::numeric_limits<double>::infinity();
  impl_->best_docking_yaw_error = std::numeric_limits<double>::infinity();
  impl_->docking_position_reached = false;
  impl_->state.controller_commanded_motion = false;
  impl_->state.safety_stopped_motion = false;
  impl_->state.requested_command = {};
  impl_->state.published_command = {};
  impl_->state.collision_monitor_action = CollisionMonitorAction::kNone;
}

void NavigationSystem::ClearGoal() {
  impl_->goal.reset(); impl_->path.clear(); impl_->state.command = {};
  impl_->state.status = NavigationStatus::kIdle;
}

void NavigationSystem::UpdateLaserScan(const Pose2d& sensor_pose, const LaserScan& scan) {
  impl_->collision_monitor.UpdateLaserScan(sensor_pose, scan);
  const auto before = impl_->costmap.digest();
  impl_->costmap.UpdateObstacleLayer(sensor_pose, scan);
  impl_->observation_changed = impl_->observation_changed || before != impl_->costmap.digest();
}

void NavigationSystem::UpdateCollisionMonitorLaserScan(
    const Pose2d& sensor_pose, const LaserScan& scan) {
  impl_->collision_monitor.UpdateLaserScan(sensor_pose, scan);
}

void NavigationSystem::UpdatePointCloud(const Pose2d& sensor_pose, const PointCloud2d& cloud) {
  impl_->collision_monitor.UpdatePointCloud(sensor_pose, cloud);
  const auto before = impl_->costmap.digest();
  impl_->costmap.UpdateObstacleLayer(sensor_pose, cloud);
  impl_->observation_changed = impl_->observation_changed || before != impl_->costmap.digest();
}

void NavigationSystem::UpdateDynamicObstacles(std::vector<PredictedObstacle> obstacles) {
  impl_->dynamic_obstacles = std::move(obstacles);
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
      std::abs(measured_velocity.linear) > 2. * impl_->config.desired_linear_velocity)
    measured_velocity.linear = impl_->state.published_command.linear;
  if (!std::isfinite(measured_velocity.angular) ||
      std::abs(measured_velocity.angular) > 2. * impl_->config.max_angular_velocity)
    measured_velocity.angular = impl_->state.published_command.angular;
  measured_velocity.linear = std::clamp(
      measured_velocity.linear, -impl_->config.max_reverse_velocity,
      impl_->config.desired_linear_velocity);
  measured_velocity.angular = std::clamp(
      measured_velocity.angular, -impl_->config.max_angular_velocity,
      impl_->config.max_angular_velocity);
  impl_->state.controller_solve_us = 0.;
  if (!impl_->goal) { impl_->state.command = {}; impl_->state.status = NavigationStatus::kIdle; return impl_->state; }
  if (!impl_->progress_initialized) {
    impl_->last_progress_time = timestamp;
    impl_->progress_initialized = true;
  }
  // A global path is a route contract, not a high-rate control signal.  The
  // local collision layer sees every scan; recomputing the same static global
  // map path while the robot moves changes its grid anchor and makes the
  // controller chase a discontinuous reference. Replan only when no route
  // exists (initial planning or after recovery invalidated it).
  if (impl_->path.empty()) {
    impl_->Replan(pose); impl_->observation_changed = false;
    impl_->last_progress_time = timestamp;
  }
  // Planning can legitimately fail while an online frontier map is changing.
  // Do not run a stale progress watchdog without a route to measure against.
  if (impl_->path.empty()) return impl_->state;
  const double goal_distance = (impl_->goal->translation() - pose.translation()).norm();
  const bool precision_goal = impl_->config.goal_xy_tolerance <= .05;
  // A centimetre-scale SE(2) docking pose is not a short global path. Switch
  // to a dedicated pose servo before the remaining polyline becomes smaller
  // than RPP's lookahead and its progress coordinate loses useful meaning.
  if (precision_goal && goal_distance <= .30) {
    impl_->state.phase = NavigationPhase::kDockToGoal;
    const double yaw_error = NormalizeAngle(Yaw(*impl_->goal) - Yaw(pose));
    if (goal_distance + .001 < impl_->best_docking_distance) {
      impl_->best_docking_distance = goal_distance;
      impl_->last_progress_time = timestamp;
      impl_->recovery_attempts = 0;
    }
    if (goal_distance <= impl_->config.goal_xy_tolerance) {
      if (!impl_->docking_position_reached) {
        impl_->docking_position_reached = true;
        impl_->best_docking_yaw_error = std::abs(yaw_error);
        impl_->last_progress_time = timestamp;
      } else if (std::abs(yaw_error) + .01 < impl_->best_docking_yaw_error) {
        impl_->best_docking_yaw_error = std::abs(yaw_error);
        impl_->last_progress_time = timestamp;
      }
    } else if (goal_distance > impl_->config.goal_xy_tolerance + .01) {
      impl_->docking_position_reached = false;
      impl_->best_docking_yaw_error = std::numeric_limits<double>::infinity();
    }
    if (goal_distance <= impl_->config.goal_xy_tolerance &&
        std::abs(yaw_error) <= impl_->config.goal_yaw_tolerance) {
      impl_->state.command = {}; impl_->state.published_command = {};
      impl_->state.status = NavigationStatus::kSucceeded;
      return impl_->state;
    }
    Twist2d docking;
    if (goal_distance <= impl_->config.goal_xy_tolerance) {
      docking.angular = std::clamp(yaw_error, -.35, .35);
    } else {
      const auto delta = impl_->goal->translation() - pose.translation();
      const double forward_error = NormalizeAngle(std::atan2(delta.y(), delta.x()) - Yaw(pose));
      // Only the final 30 cm docking servo may choose reverse, and only when
      // the home point is geometrically closer behind the base. This avoids a
      // full turn around a millimetre-scale residual while keeping all route
      // tracking forward-only.
      const bool reverse = std::abs(forward_error) > .5 * std::acos(-1.);
      const double bearing_error = reverse ?
          NormalizeAngle(forward_error + std::acos(-1.)) : forward_error;
      if (std::abs(bearing_error) > .35) {
        docking.angular = std::clamp(1.2 * bearing_error, -.35, .35);
      } else {
        const double speed = std::min(.08, std::max(.003, .4 * goal_distance));
        docking.linear = reverse ? -speed : speed;
        docking.angular = std::clamp(1.2 * bearing_error, -.35, .35);
      }
    }
    const double dv = impl_->config.max_linear_acceleration * impl_->config.control_period;
    const double dw = impl_->config.max_angular_acceleration * impl_->config.control_period;
    // Docking is closed directly on the ground-truth pose error. Slew from
    // the command actually published on the preceding cycle, not from odom's
    // yaw derivative, which can spike across angle wrapping in simulation.
    docking.linear = std::clamp(docking.linear,
        impl_->state.published_command.linear - dv,
        impl_->state.published_command.linear + dv);
    docking.angular = std::clamp(docking.angular,
        impl_->state.published_command.angular - dw,
        impl_->state.published_command.angular + dw);
    impl_->state.requested_command = docking;
    impl_->state.controller_commanded_motion =
        std::abs(docking.linear) > 1e-4 || std::abs(docking.angular) > 1e-4;
    if (DynamicCollisionImminent(pose, docking, impl_->dynamic_obstacles, impl_->config)) docking = {};
    const auto monitored = impl_->collision_monitor.Filter(pose, docking, timestamp);
    docking = monitored.command;
    impl_->state.published_command = docking;
    impl_->state.collision_monitor_action = monitored.action;
    impl_->state.safety_stopped_motion = impl_->state.controller_commanded_motion &&
        std::abs(docking.linear) <= 1e-4 && std::abs(docking.angular) <= 1e-4;
    impl_->state.minimum_ttc_s = monitored.time_to_collision_s;
    if (timestamp - impl_->last_progress_time > impl_->config.progress_timeout * 3.) {
      impl_->state.command = {};
      impl_->state.status = NavigationStatus::kBlocked;
      return impl_->state;
    }
    impl_->state.command = docking;
    impl_->state.status = NavigationStatus::kNavigating;
    return impl_->state;
  }
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
  // Translating in a circle is not navigation progress.  Measure progress in
  // the path's arc-length coordinate instead of straight-line distance to the
  // goal: a correct path around an obstacle can temporarily move farther from
  // the goal, but it never moves backward along its committed route.
  const double path_progress = PathProgress(
      impl_->path, pose, impl_->best_path_progress_m);
  impl_->state.path_progress_m = path_progress;
  if (impl_->state.phase == NavigationPhase::kAlignToPath && impl_->path.size() >= 2) {
    const auto tangent = impl_->path[1].translation() - impl_->path[0].translation();
    const double heading_error = NormalizeAngle(std::atan2(tangent.y(), tangent.x()) - Yaw(pose));
    const double alignment_score = std::abs(heading_error) + DistanceToPath(impl_->path, pose);
    if (alignment_score + .02 < impl_->best_alignment_score) {
      impl_->best_alignment_score = alignment_score;
      impl_->last_progress_time = timestamp;
    }
    if (std::abs(heading_error) < .25 || path_progress >= impl_->config.progress_radius) {
      impl_->state.phase = NavigationPhase::kTrackPath;
      impl_->last_progress_time = timestamp;
    }
  }
  if (path_progress >= impl_->best_path_progress_m + impl_->config.progress_radius) {
    impl_->best_path_progress_m = path_progress;
    impl_->last_progress_time = timestamp;
    impl_->recovery_attempts = 0;
    impl_->rotation_without_progress_rad = 0.;
  } else if (timestamp - impl_->last_progress_time > impl_->config.progress_timeout *
                 (impl_->state.phase == NavigationPhase::kAlignToPath ? 3. : 1.)) {
    ++impl_->state.recoveries; ++impl_->recovery_attempts;
    impl_->last_progress_time = timestamp;
    if (impl_->recovery_attempts > impl_->config.max_recovery_attempts) {
      impl_->state.command = {};
      impl_->state.status = NavigationStatus::kBlocked;
      return impl_->state;
    }
    // Replan from the pose actually reached. Deliberate in-place alignment is
    // supervised separately by the accumulated rotation budget below: a
    // translation-only progress watchdog must not cancel a valid alignment
    // before the controller has completed it. Scripted rotate/reverse motions
    // are not a substitute for a planner/controller-consistent route.
    impl_->path.clear();
    impl_->next_replan_time = 0.;
    impl_->state.command = {};
    return impl_->state;
  }

  Twist2d command;
  {
    const auto solve_started = std::chrono::steady_clock::now();
    command = impl_->controller->Compute(impl_->path, pose, measured_velocity, impl_->costmap,
                                         impl_->dynamic_obstacles);
    impl_->state.requested_command = command;
    impl_->state.controller_commanded_motion =
        std::abs(command.linear) > 1e-4 || std::abs(command.angular) > 1e-4;
    if (std::abs(command.linear) <= 1e-4 && std::abs(command.angular) > 1e-4)
      impl_->rotation_without_progress_rad +=
          std::abs(command.angular) * impl_->config.control_period;
    impl_->state.controller_solve_us = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - solve_started).count();
    impl_->state.controller_diagnostics = impl_->controller->Diagnostics();
    if (impl_->state.controller_diagnostics.solve_us <= 0.)
      impl_->state.controller_diagnostics.solve_us = impl_->state.controller_solve_us;
    if (DynamicCollisionImminent(pose, command, impl_->dynamic_obstacles, impl_->config))
      command = {};
    const auto monitored = impl_->collision_monitor.Filter(pose, command, timestamp);
    command = monitored.command;
    impl_->state.published_command = command;
    impl_->state.collision_monitor_action = monitored.action;
    impl_->state.safety_stopped_motion =
        impl_->state.controller_commanded_motion &&
        std::abs(command.linear) <= 1e-4 && std::abs(command.angular) <= 1e-4;
    if (!impl_->state.controller_commanded_motion && !impl_->state.safety_stopped_motion) {
      ++impl_->controller_blocked_cycles;
      if (impl_->controller_blocked_cycles >= 3) {
        ++impl_->state.recoveries;
        ++impl_->recovery_attempts;
        impl_->controller_blocked_cycles = 0;
        impl_->path.clear();
        impl_->state.command = {};
        if (impl_->recovery_attempts > impl_->config.max_recovery_attempts)
          impl_->state.status = NavigationStatus::kBlocked;
        return impl_->state;
      }
    } else {
      impl_->controller_blocked_cycles = 0;
    }
    // A differential drive may need an in-place alignment at a sharp corner,
    // but more than one complete accumulated revolution without translation
    // proves that the current route/controller pairing is not converging.
    if (impl_->rotation_without_progress_rad >= 2. * std::acos(-1.)) {
      ++impl_->state.recoveries;
      ++impl_->recovery_attempts;
      impl_->rotation_without_progress_rad = 0.;
      impl_->path.clear();
      impl_->state.command = {};
      // The pose did not translate, so replanning the identical fixed map
      // snapshot internally would reproduce the same route. Escalate once to
      // the node, which can rebuild from the newest online map.
      impl_->state.status = NavigationStatus::kBlocked;
      return impl_->state;
    }
    const double dynamic_ttc = MinimumDynamicTtc(
        pose, command, impl_->dynamic_obstacles, impl_->config);
    impl_->state.minimum_ttc_s = monitored.time_to_collision_s > 0. && dynamic_ttc > 0. ?
        std::min(monitored.time_to_collision_s, dynamic_ttc) :
        std::max(monitored.time_to_collision_s, dynamic_ttc);
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
