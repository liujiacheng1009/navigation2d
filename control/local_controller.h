#pragma once

#include "navigation2/costmap/layered_costmap.h"
#include "navigation2/types.h"

namespace navigation2d {

class LocalController {
 public:
  virtual ~LocalController() = default;
  virtual Velocity Compute(const Path& path, const Pose2d& pose, Velocity current,
                           const LayeredCostmap& costmap) const = 0;
  virtual bool CollisionImminent(const Pose2d& pose, Velocity command,
                                 const LayeredCostmap& costmap) const = 0;
};

}  // namespace navigation2d
