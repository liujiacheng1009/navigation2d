// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2025 Open Navigation LLC
// Adapted for Navigation2D's ROS-free API under the MIT license.
#include "navigation2d/control/nav2_mppi_core.h"

#include <algorithm>
#include <cmath>

namespace navigation2d::control_internal {

Nav2MppiCore::Nav2MppiCore(const NavigationConfig& config)
    : config_(config),
      sequence_velocity_(Eigen::ArrayXd::Zero(config.mppi_time_steps)),
      sequence_angular_(Eigen::ArrayXd::Zero(config.mppi_time_steps)),
      random_(config.mppi_seed) {}

void Nav2MppiCore::Reset() {
  sequence_velocity_.setZero();
  sequence_angular_.setZero();
  history_.fill({});
}

void Nav2MppiCore::BlendProposal(Twist2d proposal) {
  sequence_velocity_ = .85 * sequence_velocity_ + .15 * proposal.linear;
  sequence_angular_ = .85 * sequence_angular_ + .15 * proposal.angular;
}

MppiRollouts Nav2MppiCore::Generate(const Pose2d& pose, Twist2d current) {
  const int batch = config_.mppi_batch_size, steps = config_.mppi_time_steps;
  MppiRollouts result{
      Eigen::ArrayXXd(batch, steps), Eigen::ArrayXXd(batch, steps),
      Eigen::ArrayXXd(batch, steps), Eigen::ArrayXXd(batch, steps),
      Eigen::ArrayXXd(batch, steps), Eigen::ArrayXXd(batch, steps),
      Eigen::ArrayXXd(batch, steps)};
  std::normal_distribution<double> normal(0., 1.);
  for (int sample = 0; sample < batch; ++sample) for (int step = 0; step < steps; ++step) {
    result.noise_velocity(sample, step) = normal(random_) * config_.mppi_vx_std;
    result.noise_angular(sample, step) = normal(random_) * config_.mppi_wz_std;
  }
  result.velocity = result.noise_velocity.rowwise() + sequence_velocity_.transpose();
  result.angular = result.noise_angular.rowwise() + sequence_angular_.transpose();

  Eigen::ArrayXd x = Eigen::ArrayXd::Constant(batch, X(pose));
  Eigen::ArrayXd y = Eigen::ArrayXd::Constant(batch, Y(pose));
  Eigen::ArrayXd yaw = Eigen::ArrayXd::Constant(batch, Yaw(pose));
  Eigen::ArrayXd previous_v = Eigen::ArrayXd::Constant(batch, current.linear);
  Eigen::ArrayXd previous_w = Eigen::ArrayXd::Constant(batch, current.angular);
  const double dv = config_.max_linear_acceleration * config_.control_period;
  const double dw = config_.max_angular_acceleration * config_.control_period;
  for (int step = 0; step < steps; ++step) {
    result.velocity.col(step) = result.velocity.col(step)
        .max((previous_v - dv).max(-config_.max_reverse_velocity))
        .min((previous_v + dv).min(config_.desired_linear_velocity));
    result.angular.col(step) = result.angular.col(step)
        .max((previous_w - dw).max(-config_.max_angular_velocity))
        .min((previous_w + dw).min(config_.max_angular_velocity));
    const Eigen::ArrayXd next_yaw = yaw + result.angular.col(step) * config_.control_period;
    const auto straight = result.angular.col(step).abs() < 1e-9;
    const Eigen::ArrayXd radius = result.velocity.col(step) /
        result.angular.col(step).unaryExpr([](double value) {
          return std::abs(value) < 1e-9 ? 1. : value;
        });
    x += straight.select(result.velocity.col(step) * yaw.cos() * config_.control_period,
                         radius * (next_yaw.sin() - yaw.sin()));
    y += straight.select(result.velocity.col(step) * yaw.sin() * config_.control_period,
                         -radius * (next_yaw.cos() - yaw.cos()));
    yaw = next_yaw;
    result.x.col(step) = x; result.y.col(step) = y; result.yaw.col(step) = yaw;
    previous_v = result.velocity.col(step); previous_w = result.angular.col(step);
  }
  return result;
}

bool Nav2MppiCore::Update(const MppiRollouts& rollouts, const Eigen::ArrayXd& costs) {
  const double minimum = costs.minCoeff();
  Eigen::ArrayXd weights = (-(costs - minimum) / config_.mppi_temperature).exp();
  const double normalization = weights.sum();
  if (!std::isfinite(normalization) || normalization <= 1e-12) return false;
  weights /= normalization;
  sequence_velocity_ +=
      (rollouts.noise_velocity.matrix().transpose() * weights.matrix()).array();
  sequence_angular_ +=
      (rollouts.noise_angular.matrix().transpose() * weights.matrix()).array();
  return true;
}

Twist2d Nav2MppiCore::Command(Twist2d current) {
  constexpr std::array<double, 9> coefficients{
      -21. / 231., 14. / 231., 39. / 231., 54. / 231., 59. / 231.,
       54. / 231., 39. / 231., 14. / 231., -21. / 231.};
  const Eigen::ArrayXd original_v = sequence_velocity_, original_w = sequence_angular_;
  const auto sample = [&](const Eigen::ArrayXd& sequence, int index, bool angular) {
    if (index < 0) {
      const auto& command = history_[static_cast<std::size_t>(index + 4)];
      return angular ? command.angular : command.linear;
    }
    return sequence(std::min(index, static_cast<int>(sequence.size()) - 1));
  };
  for (int step = 0; step + 1 < config_.mppi_time_steps; ++step) {
    double velocity = 0., angular = 0.;
    for (int offset = -4; offset <= 4; ++offset) {
      const double coefficient = coefficients[static_cast<std::size_t>(offset + 4)];
      velocity += coefficient * sample(original_v, step + offset, false);
      angular += coefficient * sample(original_w, step + offset, true);
    }
    sequence_velocity_(step) = velocity; sequence_angular_(step) = angular;
  }
  const double dv = config_.max_linear_acceleration * config_.control_period;
  const double dw = config_.max_angular_acceleration * config_.control_period;
  Twist2d command{
      std::clamp(sequence_velocity_(0),
                 std::max(-config_.max_reverse_velocity, current.linear - dv),
                 std::min(config_.desired_linear_velocity, current.linear + dv)),
      std::clamp(sequence_angular_(0),
                 std::max(-config_.max_angular_velocity, current.angular - dw),
                 std::min(config_.max_angular_velocity, current.angular + dw))};
  std::move(history_.begin() + 1, history_.end(), history_.begin());
  history_.back() = command;
  for (Eigen::Index index = 0; index + 1 < sequence_velocity_.size(); ++index) {
    sequence_velocity_(index) = sequence_velocity_(index + 1);
    sequence_angular_(index) = sequence_angular_(index + 1);
  }
  sequence_velocity_(sequence_velocity_.size() - 1) = sequence_velocity_(sequence_velocity_.size() - 2);
  sequence_angular_(sequence_angular_.size() - 1) = sequence_angular_(sequence_angular_.size() - 2);
  return command;
}

}  // namespace navigation2d::control_internal
