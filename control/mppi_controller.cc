// Copyright (c) 2022 Samsung Research America
// Copyright (c) 2025 Open Navigation LLC
// SPDX-License-Identifier: MIT
#include "navigation2d/control/mppi_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "navigation2d/control/dynamic_safety.h"
#include "navigation2d/control/nav2_mppi_core.h"

namespace navigation2d {
namespace {

double NormalizeAngle(double value) { return std::atan2(std::sin(value), std::cos(value)); }

Pose2d Integrate(const Pose2d& pose, const Twist2d& control, double dt) {
  const double yaw = Yaw(pose), next_yaw = NormalizeAngle(yaw + control.angular * dt);
  if (std::abs(control.angular) < 1e-9)
    return MakePose2d(X(pose) + control.linear * std::cos(yaw) * dt,
                      Y(pose) + control.linear * std::sin(yaw) * dt, next_yaw);
  const double radius = control.linear / control.angular;
  return MakePose2d(X(pose) + radius * (std::sin(next_yaw) - std::sin(yaw)),
                    Y(pose) - radius * (std::cos(next_yaw) - std::cos(yaw)), next_yaw);
}

std::size_t NearestPathPoint(const Path& path, const Pose2d& pose) {
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double distance = (path[index].translation() - pose.translation()).squaredNorm();
    if (distance < best) { best = distance; nearest = index; }
  }
  return nearest;
}

double PathHeading(const Path& path, std::size_t index) {
  if (path.size() < 2) return Yaw(path.back());
  const std::size_t next = std::min(index + 1, path.size() - 1);
  const std::size_t previous = next == index ? index - 1 : index;
  const auto delta = path[next].translation() - path[previous].translation();
  return std::atan2(delta.y(), delta.x());
}

}  // namespace

MppiController::MppiController(NavigationConfig config)
    : config_(std::move(config)),
      core_(std::make_unique<control_internal::Nav2MppiCore>(config_)) {}
MppiController::~MppiController() = default;

Twist2d MppiController::Compute(const Path& path, const Pose2d& pose, Twist2d current,
                                const LayeredCostmap& costmap,
                                const std::vector<PredictedObstacle>& dynamic_obstacles) const {
  diagnostics_ = {};
  diagnostics_.backend = ControllerBackend::kMppi;
  diagnostics_.status = ControllerSolveStatus::kSuccess;
  if (path.empty()) return {};
  const std::size_t robot_nearest = NearestPathPoint(path, pose);
  const std::size_t proposal_index = std::min(robot_nearest + 8, path.size() - 1);
  const auto proposal_delta = path[proposal_index].translation() - pose.translation();
  const double proposal_error = NormalizeAngle(
      std::atan2(proposal_delta.y(), proposal_delta.x()) - Yaw(pose));
  core_->BlendProposal({
      std::abs(proposal_error) > .6 ? 0. :
          config_.desired_linear_velocity * std::max(.25, 1. - std::abs(proposal_error)),
      std::clamp(2. * proposal_error, -config_.max_angular_velocity,
                 config_.max_angular_velocity)});
  const double robot_goal_distance = (path.back().translation() - pose.translation()).norm();

  for (int iteration = 0; iteration < config_.mppi_iterations; ++iteration) {
    const auto rollouts = core_->Generate(pose, current);
    Eigen::ArrayXd costs = Eigen::ArrayXd::Zero(config_.mppi_batch_size);
    for (int sample = 0; sample < config_.mppi_batch_size; ++sample) {
      double path_align = 0., path_angle_sum = 0., obstacle_cost = 0., forward = 0.;
      double smoothness = 0.;
      bool collision = false;
      for (int step = 0; step < config_.mppi_time_steps; ++step) {
        const Pose2d projected = MakePose2d(
            rollouts.x(sample, step), rollouts.y(sample, step), rollouts.yaw(sample, step));
        const auto [cx, cy] = costmap.grid().ToCell(X(projected), Y(projected));
        const double normalized_cost = static_cast<double>(costmap.cost(cx, cy)) / 252.;
        obstacle_cost += normalized_cost * normalized_cost;
        collision = costmap.lethal(X(projected), Y(projected), config_.robot_radius) ||
            DynamicCollisionAt(projected, (step + 1) * config_.control_period,
                               dynamic_obstacles, config_);
        if (collision) break;
        if (step % 3 == 0) {
          const std::size_t nearest = NearestPathPoint(path, projected);
          path_align += (path[nearest].translation() - projected.translation()).norm();
          path_angle_sum += std::abs(NormalizeAngle(
              PathHeading(path, nearest) - Yaw(projected)));
        }
        const double previous_v = step == 0 ? current.linear : rollouts.velocity(sample, step - 1);
        const double previous_w = step == 0 ? current.angular : rollouts.angular(sample, step - 1);
        smoothness += std::abs(rollouts.velocity(sample, step) - previous_v) +
                      .25 * std::abs(rollouts.angular(sample, step) - previous_w);
        forward += std::max(0., -rollouts.velocity(sample, step));
        costs(sample) += config_.mppi_gamma *
            ((rollouts.velocity(sample, step) - rollouts.noise_velocity(sample, step)) *
                 rollouts.noise_velocity(sample, step) /
                 (config_.mppi_vx_std * config_.mppi_vx_std) +
             (rollouts.angular(sample, step) - rollouts.noise_angular(sample, step)) *
                 rollouts.noise_angular(sample, step) /
                 (config_.mppi_wz_std * config_.mppi_wz_std));
      }
      if (collision) { costs(sample) += 1e6; continue; }
      const int last = config_.mppi_time_steps - 1;
      const Pose2d terminal = MakePose2d(
          rollouts.x(sample, last), rollouts.y(sample, last), rollouts.yaw(sample, last));
      const std::size_t nearest = NearestPathPoint(path, terminal);
      const std::size_t follow = std::min(nearest + 8, path.size() - 1);
      const double goal = robot_goal_distance < 1.4 ?
          (path.back().translation() - terminal.translation()).norm() : 0.;
      const double goal_angle = robot_goal_distance < .5 ?
          std::abs(NormalizeAngle(Yaw(path.back()) - Yaw(terminal))) : 0.;
      costs(sample) += config_.mppi_cost_weight * obstacle_cost +
          config_.mppi_goal_weight * goal + config_.mppi_goal_angle_weight * goal_angle +
          config_.mppi_path_align_weight * path_align /
              std::max(1, config_.mppi_time_steps / 3) +
          config_.mppi_path_follow_weight *
              (path[follow].translation() - terminal.translation()).norm() +
          config_.mppi_path_angle_weight * path_angle_sum /
              std::max(1, config_.mppi_time_steps / 3) +
          config_.mppi_prefer_forward_weight * forward +
          config_.mppi_smoothness_weight * smoothness;
    }
    if (!core_->Update(rollouts, costs)) return {};
  }
  const Twist2d command = core_->Command(current);
  return CollisionImminent(pose, command, costmap) ||
      DynamicCollisionImminent(pose, command, dynamic_obstacles, config_) ? Twist2d{} : command;
}

bool MppiController::CollisionImminent(const Pose2d& pose, Twist2d command,
                                       const LayeredCostmap& costmap) const {
  Pose2d projected = pose;
  for (double time = 0.; time < config_.collision_horizon; time += config_.control_period) {
    projected = Integrate(projected, command, config_.control_period);
    if (costmap.lethal(X(projected), Y(projected), config_.robot_radius)) return true;
  }
  return false;
}

}  // namespace navigation2d
