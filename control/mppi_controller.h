#pragma once

#include <array>
#include <random>
#include <vector>

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/control/local_controller.h"

namespace navigation2d {

// ROS-free differential-drive MPPI controller adapted from Nav2's
// nav2_mppi_controller optimizer and critic architecture.
class MppiController final : public LocalController {
 public:
  explicit MppiController(NavigationConfig config);
  Twist2d Compute(const Path& path, const Pose2d& pose, Twist2d current,
                  const LayeredCostmap& costmap,
                  const std::vector<PredictedObstacle>& dynamic_obstacles = {}) const override;
  bool CollisionImminent(const Pose2d& pose, Twist2d command,
                         const LayeredCostmap& costmap) const override;

 private:
  NavigationConfig config_;
  mutable std::vector<Twist2d> control_sequence_;
  mutable std::array<Twist2d, 4> control_history_{};
  mutable std::mt19937 random_;
};

}  // namespace navigation2d
