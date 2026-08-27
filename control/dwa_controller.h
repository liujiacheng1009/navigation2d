#pragma once

#include "navigation2/application/navigation_config.h"
#include "navigation2/control/local_controller.h"

namespace navigation2d {

class DwaController final : public LocalController {
 public:
  explicit DwaController(NavigationConfig config) : config_(config) {}
  Velocity Compute(const Path& path, const Pose2d& pose, Velocity current,
                   const LayeredCostmap& costmap) const override;
  bool CollisionImminent(const Pose2d& pose, Velocity command,
                         const LayeredCostmap& costmap) const override;
 private:
  NavigationConfig config_;
};

}  // namespace navigation2d
