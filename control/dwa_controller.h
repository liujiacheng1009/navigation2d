#pragma once

#include "navigation2d/control/dwa_parameters.h"
#include "navigation2d/control/local_controller.h"

namespace navigation2d {

class DwaController final : public LocalController {
 public:
  explicit DwaController(const NavigationConfig& config);
  explicit DwaController(DwaParameters config) : config_(config) {}
  Twist2d Compute(const Path& path, const Pose2d& pose, Twist2d current,
                  const LayeredCostmap& costmap,
                  const std::vector<PredictedObstacle>& dynamic_obstacles = {}) const override;
  bool CollisionImminent(const Pose2d& pose, Twist2d command,
                         const LayeredCostmap& costmap) const override;
  ControllerDiagnostics Diagnostics() const override {
    ControllerDiagnostics diagnostics;
    diagnostics.backend = ControllerBackend::kDwa;
    diagnostics.status = ControllerSolveStatus::kSuccess;
    return diagnostics;
  }

 private:
  DwaParameters config_;
};

}  // namespace navigation2d
