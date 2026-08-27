#include "navigation2/application/navigation_system.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>
#include "navigation2/control/regulated_pure_pursuit.h"
#include "navigation2/control/dwa_controller.h"
#include "navigation2/costmap/grid_2d.h"
#include "navigation2/costmap/layered_costmap.h"
#include "navigation2/planning/global_planner.h"
#include "navigation2/planning/planner_factory.h"

namespace navigation2d {
namespace {
double NormalizeAngle(double value) { return std::atan2(std::sin(value), std::cos(value)); }
std::unique_ptr<LocalController> MakeController(const NavigationConfig& config) {
  if (config.controller == "rpp") return std::make_unique<RegulatedPurePursuit>(config);
  if (config.controller == "dwa") return std::make_unique<DwaController>(config);
  throw std::runtime_error("unknown controller: " + config.controller);
}
}

class NavigationSystem::Impl {
 public:
  Impl(NavigationConfig value, const std::string& map_path)
      : config(std::move(value)), costmap(Grid2d::Load(map_path), config),
        planner(MakeGlobalPlanner(config.planner, 0.)), controller(MakeController(config)) {
    if (std::abs(costmap.grid().resolution() - config.map_resolution) > 1e-9)
      throw std::runtime_error("map resolution does not match navigation configuration");
  }

  void Replan(const Pose2d& pose) {
    try {
      path = planner->Plan(costmap, pose, *goal); ++state.replans;
      double path_length = 0.;
      for (std::size_t i = 1; i < path.size(); ++i)
        path_length += std::hypot(path[i].x - path[i - 1].x,
                                  path[i].y - path[i - 1].y);
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

NavigationState NavigationSystem::ComputeCommand(const Pose2d& pose, Velocity measured_velocity,
                                                 double timestamp) {
  if (!impl_->goal) { impl_->state.command = {}; impl_->state.status = NavigationStatus::kIdle; return impl_->state; }
  if (!impl_->progress_initialized) {
    impl_->last_progress_pose = pose; impl_->last_progress_time = timestamp;
    impl_->progress_initialized = true;
  }
  if (impl_->observation_changed || impl_->path.empty() || timestamp >= impl_->next_replan_time) {
    impl_->Replan(pose); impl_->observation_changed = false;
    impl_->next_replan_time = timestamp + impl_->config.global_replan_period;
  }
  const double goal_distance = std::hypot(impl_->goal->x - pose.x, impl_->goal->y - pose.y);
  if (goal_distance <= impl_->config.goal_xy_tolerance) {
    const double yaw_error = NormalizeAngle(impl_->goal->yaw - pose.yaw);
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
  if (std::hypot(pose.x - impl_->last_progress_pose.x, pose.y - impl_->last_progress_pose.y) >=
      impl_->config.progress_radius) {
    impl_->last_progress_pose = pose; impl_->last_progress_time = timestamp;
  } else if (timestamp - impl_->last_progress_time > impl_->config.progress_timeout) {
    ++impl_->state.recoveries; impl_->last_progress_time = timestamp;
    impl_->recovery_steps_remaining = static_cast<int>(
        std::ceil(impl_->config.recovery_duration / impl_->config.control_period));
  }
  if (impl_->path.empty() && impl_->recovery_steps_remaining == 0) return impl_->state;

  Velocity command;
  if (impl_->recovery_steps_remaining > 0) {
    command = {std::max(-impl_->config.max_reverse_velocity,
                        impl_->config.recovery_linear_velocity), 0.};
    if (impl_->controller->CollisionImminent(pose, command, impl_->costmap))
      command = {0., impl_->config.recovery_angular_velocity};
    --impl_->recovery_steps_remaining;
  } else {
    command = impl_->controller->Compute(impl_->path, pose, measured_velocity, impl_->costmap);
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
