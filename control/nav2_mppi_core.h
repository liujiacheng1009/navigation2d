#pragma once

#include <array>
#include <random>

#include <Eigen/Core>

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/control/local_controller.h"

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
  void BlendProposal(Twist2d proposal);
  MppiRollouts Generate(const Pose2d& pose, Twist2d current);
  bool Update(const MppiRollouts& rollouts, const Eigen::ArrayXd& costs);
  Twist2d Command(Twist2d current);

 private:
  NavigationConfig config_;
  Eigen::ArrayXd sequence_velocity_, sequence_angular_;
  std::array<Twist2d, 4> history_{};
  std::mt19937 random_;
};

}  // namespace navigation2d::control_internal
