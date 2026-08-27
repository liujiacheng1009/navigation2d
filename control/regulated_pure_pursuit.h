#pragma once

#include "navigation2/types.h"
#include "navigation2/application/navigation_config.h"
#include "navigation2/costmap/layered_costmap.h"

namespace navigation2d {

class RegulatedPurePursuit {
 public:
  explicit RegulatedPurePursuit(NavigationConfig config) : config_(config) {}
  Velocity Compute(const Path& path, const Pose2d& pose, Velocity current,
                   const LayeredCostmap& costmap) const;
  bool CollisionImminent(const Pose2d& pose, Velocity command,
                         const LayeredCostmap& costmap) const;

 private:
  NavigationConfig config_;
};

}  // namespace navigation2d
