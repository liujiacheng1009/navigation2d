#pragma once

#include <cstddef>
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
  // Progress is monotone along one global path. This prevents a closest-point
  // query from snapping to an already traversed, spatially nearby segment.
  mutable const void* active_path_data_ = nullptr;
  mutable std::size_t active_path_size_ = 0;
  mutable Pose2d active_path_start_;
  mutable Pose2d active_path_goal_;
  mutable std::size_t committed_path_index_ = 0;
};

}  // namespace navigation2d
