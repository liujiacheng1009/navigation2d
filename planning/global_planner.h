#pragma once

#include "navigation2d/costmap/layered_costmap.h"
#include "navigation2d/types.h"

namespace navigation2d {

class GlobalPlanner {
 public:
  virtual ~GlobalPlanner() = default;
  virtual Path Plan(const LayeredCostmap& costmap, const Pose2d& start,
                    const Pose2d& goal) const = 0;
};

}  // namespace navigation2d
