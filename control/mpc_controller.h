#pragma once

#include <memory>
#include <vector>

#include "navigation2d/application/navigation_config.h"
#include "navigation2d/control/local_controller.h"

namespace navigation2d {

class AcadosMpcBackend;

// Deterministic constrained MPC / MPCC controller.  It intentionally owns no
// ROS or solver dependency: its problem definition matches a future generated
// acados backend, while this shooting backend is usable in the core library.
class MpcController final : public LocalController {
 public:
  explicit MpcController(NavigationConfig config);
  ~MpcController() override;
  Twist2d Compute(const Path& path, const Pose2d& pose, Twist2d current,
                  const LayeredCostmap& costmap,
                  const std::vector<PredictedObstacle>& dynamic_obstacles = {}) const override;
  bool CollisionImminent(const Pose2d& pose, Twist2d command,
                         const LayeredCostmap& costmap) const override;

 private:
  NavigationConfig config_;
  mutable std::unique_ptr<AcadosMpcBackend> acados_;
};

}  // namespace navigation2d
