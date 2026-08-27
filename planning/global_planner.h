#pragma once

#include <vector>

#include "navigation2d/costmap/layered_costmap.h"
#include "navigation2d/geometry/pose_2d.h"

namespace navigation2d {

using Path = std::vector<Pose2d>;

class GlobalPlanner {
 public:
  virtual ~GlobalPlanner() = default;
  virtual Path Plan(const LayeredCostmap& costmap, const Pose2d& start,
                    const Pose2d& goal) const = 0;
};

}  // namespace navigation2d
