#pragma once

#include <memory>

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/control/local_controller.h"

namespace navigation2d {

namespace control_internal { class Nav2MppiCore; }

// ROS-free differential-drive MPPI controller adapted from Nav2's
// nav2_mppi_controller optimizer and critic architecture.
class MppiController final : public LocalController {
 public:
  explicit MppiController(NavigationConfig config);
  ~MppiController() override;
  Twist2d Compute(const Path& path, const Pose2d& pose, Twist2d current,
                  const LayeredCostmap& costmap,
                  const std::vector<PredictedObstacle>& dynamic_obstacles = {}) const override;
  bool CollisionImminent(const Pose2d& pose, Twist2d command,
                         const LayeredCostmap& costmap) const override;
  ControllerDiagnostics Diagnostics() const override { return diagnostics_; }

 private:
  NavigationConfig config_;
  mutable std::unique_ptr<control_internal::Nav2MppiCore> core_;
  mutable ControllerDiagnostics diagnostics_;
};

}  // namespace navigation2d
