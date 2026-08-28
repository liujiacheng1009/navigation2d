#pragma once

#include "navigation2d/control/local_controller.h"
#include "navigation2d/application/navigation_config.h"
#include "navigation2d/control/local_controller.h"

namespace navigation2d {

class RegulatedPurePursuit final : public LocalController {
 public:
  explicit RegulatedPurePursuit(NavigationConfig config) : config_(config) {}
  Twist2d Compute(const Path& path, const Pose2d& pose, Twist2d current,
                  const LayeredCostmap& costmap,
                  const std::vector<PredictedObstacle>& dynamic_obstacles = {}) const override;
  bool CollisionImminent(const Pose2d& pose, Twist2d command,
                         const LayeredCostmap& costmap) const override;
  ControllerDiagnostics Diagnostics() const override {
    ControllerDiagnostics diagnostics;
    diagnostics.backend = ControllerBackend::kRpp;
    diagnostics.status = ControllerSolveStatus::kSuccess;
    return diagnostics;
  }

 private:
  NavigationConfig config_;
};

}  // namespace navigation2d
