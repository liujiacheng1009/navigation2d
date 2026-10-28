#pragma once
#include <Eigen/Core>
#include <array>
#include <concepts>
#include <random>
#include <utility>

#include "navigation2d/control/local_controller.h"
#include "navigation2d/control/mppi_parameters.h"

namespace navigation2d::control_internal {

struct MppiRollouts {
  Eigen::ArrayXXd x, y, yaw;
  Eigen::ArrayXXd velocity, angular;
  Eigen::ArrayXXd noise_velocity, noise_angular;
};

// Numerical core adapted from Nav2 nav2_mppi_controller optimizer at
// a143bcf9860273421f1918e525fc617af947c009. ROS and plugin critics remain in
// the adapter; batch rollout, softmax, sequence shift and SG filtering remain here.
class Nav2MppiCore {
 public:
  explicit Nav2MppiCore(const NavigationConfig& config);
  // Constrained deduction preserves the legacy Constructor({}) entry point.
  template <std::same_as<MppiRolloutParameters> Parameters>
  explicit Nav2MppiCore(Parameters config) : Nav2MppiCore(std::move(config), ParameterTag{}) {}
  // A warm start belongs to exactly one geometric reference path. Retaining
  // it across a replan injects steering commands for an unrelated route.
  void Reset();
  void BlendProposal(Twist2d proposal);
  MppiRollouts Generate(const Pose2d& pose, Twist2d current);
  bool Update(const MppiRollouts& rollouts, const Eigen::ArrayXd& costs);
  Twist2d Command(Twist2d current);

 private:
  struct ParameterTag {};
  explicit Nav2MppiCore(MppiRolloutParameters config, ParameterTag);
  MppiRolloutParameters config_;
  Eigen::ArrayXd sequence_velocity_, sequence_angular_;
  std::array<Twist2d, 4> history_{};
  std::mt19937 random_;
};

}  // namespace navigation2d::control_internal
