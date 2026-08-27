// Copyright (c) 2022 Samsung Research America
// Copyright (c) 2025 Open Navigation LLC
// SPDX-License-Identifier: Apache-2.0

#include "navigation2d/control/mppi_controller.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation2d {
namespace {

double NormalizeAngle(double value) { return std::atan2(std::sin(value), std::cos(value)); }

Pose2d Integrate(const Pose2d& pose, const Twist2d& control, double dt) {
  const double yaw = Yaw(pose);
  const double next_yaw = NormalizeAngle(yaw + control.angular * dt);
  if (std::abs(control.angular) < 1e-9) {
    return MakePose2d(X(pose) + control.linear * std::cos(yaw) * dt,
                      Y(pose) + control.linear * std::sin(yaw) * dt, next_yaw);
  }
  const double radius = control.linear / control.angular;
  return MakePose2d(X(pose) + radius * (std::sin(next_yaw) - std::sin(yaw)),
                    Y(pose) - radius * (std::cos(next_yaw) - std::cos(yaw)), next_yaw);
}

std::size_t NearestPathPoint(const Path& path, const Pose2d& pose) {
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double distance = (path[i].translation() - pose.translation()).squaredNorm();
    if (distance < best) { best = distance; nearest = i; }
  }
  return nearest;
}

double PathHeading(const Path& path, std::size_t index) {
  if (path.size() < 2) return Yaw(path.back());
  const std::size_t next = std::min(index + 1, path.size() - 1);
  const std::size_t previous = next == index ? index - 1 : index;
  const Eigen::Vector2d delta = path[next].translation() - path[previous].translation();
  return std::atan2(delta.y(), delta.x());
}

}  // namespace

MppiController::MppiController(NavigationConfig config)
    : config_(std::move(config)),
      control_sequence_(static_cast<std::size_t>(config_.mppi_time_steps)),
      random_(config_.mppi_seed) {}

Twist2d MppiController::Compute(const Path& path, const Pose2d& pose, Twist2d current,
                                const LayeredCostmap& costmap) const {
  if (path.empty()) return {};
  const int batch = config_.mppi_batch_size;
  const int steps = config_.mppi_time_steps;
  Eigen::ArrayXXd noise_v(batch, steps), noise_w(batch, steps);
  Eigen::ArrayXd costs(batch), weights(batch);
  std::normal_distribution<double> normal(0., 1.);
  const std::size_t robot_nearest = NearestPathPoint(path, pose);
  const std::size_t proposal_index = std::min(robot_nearest + 8, path.size() - 1);
  const Eigen::Vector2d proposal_delta = path[proposal_index].translation() - pose.translation();
  const double proposal_heading = std::atan2(proposal_delta.y(), proposal_delta.x());
  const double proposal_error = NormalizeAngle(proposal_heading - Yaw(pose));
  const Twist2d proposal{
      std::abs(proposal_error) > .6 ? 0. :
          config_.desired_linear_velocity * std::max(.25, 1. - std::abs(proposal_error)),
      std::clamp(2. * proposal_error, -config_.max_angular_velocity,
                 config_.max_angular_velocity)};
  for (auto& control : control_sequence_) {
    control.linear = .85 * control.linear + .15 * proposal.linear;
    control.angular = .85 * control.angular + .15 * proposal.angular;
  }
  const double robot_goal_distance = (path.back().translation() - pose.translation()).norm();

  for (int iteration = 0; iteration < config_.mppi_iterations; ++iteration) {
    noise_v.setZero();
    noise_w.setZero();
    costs.setZero();
    for (int sample = 0; sample < batch; ++sample) {
      Pose2d projected = pose;
      Twist2d previous = current;
      double path_align = 0., path_angle_sum = 0., obstacle = 0., forward = 0.;
      double smoothness = 0., constraint = 0.;
      bool collision = false;
      for (int step = 0; step < steps; ++step) {
        const double nv = normal(random_) * config_.mppi_vx_std;
        const double nw = normal(random_) * config_.mppi_wz_std;
        noise_v(sample, step) = nv;
        noise_w(sample, step) = nw;
        const double raw_v = control_sequence_[step].linear + nv;
        const double raw_w = control_sequence_[step].angular + nw;
        const double dv = config_.max_linear_acceleration * config_.control_period;
        const double dw = config_.max_angular_acceleration * config_.control_period;
        Twist2d control{
            std::clamp(raw_v, std::max(-config_.max_reverse_velocity, previous.linear - dv),
                       std::min(config_.desired_linear_velocity, previous.linear + dv)),
            std::clamp(raw_w, std::max(-config_.max_angular_velocity, previous.angular - dw),
                       std::min(config_.max_angular_velocity, previous.angular + dw))};
        constraint += std::abs(raw_v - control.linear) + std::abs(raw_w - control.angular);
        smoothness += std::abs(control.linear - previous.linear) +
                      .25 * std::abs(control.angular - previous.angular);
        forward += std::max(0., -control.linear);
        projected = Integrate(projected, control, config_.control_period);
        const auto [cx, cy] = costmap.grid().ToCell(X(projected), Y(projected));
        const double normalized_cost = static_cast<double>(costmap.cost(cx, cy)) / 252.;
        obstacle += normalized_cost * normalized_cost;
        if (costmap.lethal(X(projected), Y(projected), config_.robot_radius)) {
          collision = true;
          break;
        }
        if (step % 3 == 0) {
          const std::size_t nearest = NearestPathPoint(path, projected);
          path_align += (path[nearest].translation() - projected.translation()).norm();
          path_angle_sum += std::abs(
              NormalizeAngle(PathHeading(path, nearest) - Yaw(projected)));
        }
        costs(sample) += config_.mppi_gamma *
            (control_sequence_[step].linear * nv / (config_.mppi_vx_std * config_.mppi_vx_std) +
             control_sequence_[step].angular * nw / (config_.mppi_wz_std * config_.mppi_wz_std));
        previous = control;
      }
      if (collision) {
        costs(sample) += 1e6;
        continue;
      }
      const std::size_t nearest = NearestPathPoint(path, projected);
      const std::size_t follow_index = std::min(nearest + 8, path.size() - 1);
      const double path_follow =
          (path[follow_index].translation() - projected.translation()).norm();
      const double path_angle = path_angle_sum / std::max(1, steps / 3);
      const double goal = robot_goal_distance < 1.4 ?
          (path.back().translation() - projected.translation()).norm() : 0.;
      const double goal_angle = robot_goal_distance < .5 ?
          std::abs(NormalizeAngle(Yaw(path.back()) - Yaw(projected))) : 0.;
      costs(sample) += config_.mppi_constraint_weight * constraint +
          config_.mppi_cost_weight * obstacle + config_.mppi_goal_weight * goal +
          config_.mppi_goal_angle_weight * goal_angle +
          config_.mppi_path_align_weight * path_align / std::max(1, steps / 3) +
          config_.mppi_path_follow_weight * path_follow +
          config_.mppi_path_angle_weight * path_angle +
          config_.mppi_prefer_forward_weight * forward +
          config_.mppi_smoothness_weight * smoothness;
    }

    const double minimum = costs.minCoeff();
    weights = (-(costs - minimum) / config_.mppi_temperature).exp();
    const double normalization = weights.sum();
    if (!std::isfinite(normalization) || normalization <= 1e-12) return {};
    weights /= normalization;
    for (int step = 0; step < steps; ++step) {
      control_sequence_[step].linear += (weights * noise_v.col(step)).sum();
      control_sequence_[step].angular += (weights * noise_w.col(step)).sum();
    }
  }

  // Nav2's current quadratic Savitzky-Golay filter: 9-point window and four
  // historical commands for the leading edge.
  constexpr std::array<double, 9> coefficients{
      -21. / 231., 14. / 231., 39. / 231., 54. / 231., 59. / 231.,
       54. / 231., 39. / 231., 14. / 231., -21. / 231.};
  const std::vector<Twist2d> unfiltered = control_sequence_;
  const auto sample = [&](int index) -> const Twist2d& {
    if (index < 0) return control_history_[static_cast<std::size_t>(index + 4)];
    return unfiltered[static_cast<std::size_t>(
        std::min(index, static_cast<int>(unfiltered.size()) - 1))];
  };
  for (int step = 0; step + 1 < config_.mppi_time_steps; ++step) {
    Twist2d filtered;
    for (int offset = -4; offset <= 4; ++offset) {
      filtered.linear += coefficients[static_cast<std::size_t>(offset + 4)] *
                         sample(step + offset).linear;
      filtered.angular += coefficients[static_cast<std::size_t>(offset + 4)] *
                          sample(step + offset).angular;
    }
    control_sequence_[static_cast<std::size_t>(step)] = filtered;
  }
  const double dv = config_.max_linear_acceleration * config_.control_period;
  const double dw = config_.max_angular_acceleration * config_.control_period;
  Twist2d command{
      std::clamp(control_sequence_.front().linear,
                 std::max(-config_.max_reverse_velocity, current.linear - dv),
                 std::min(config_.desired_linear_velocity, current.linear + dv)),
      std::clamp(control_sequence_.front().angular,
                 std::max(-config_.max_angular_velocity, current.angular - dw),
                 std::min(config_.max_angular_velocity, current.angular + dw))};
  control_sequence_.front() = command;
  std::move(control_history_.begin() + 1, control_history_.end(), control_history_.begin());
  control_history_.back() = command;
  std::move(control_sequence_.begin() + 1, control_sequence_.end(), control_sequence_.begin());
  control_sequence_.back() = control_sequence_[control_sequence_.size() - 2];
  return CollisionImminent(pose, command, costmap) ? Twist2d{} : command;
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
