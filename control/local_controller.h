#pragma once

#include "navigation2d/costmap/layered_costmap.h"
#include "navigation2d/control/obstacle_prediction.h"
#include "navigation2d/planning/global_planner.h"

namespace navigation2d {

struct Twist2d {
  double linear = 0.;
  double angular = 0.;
};

class LocalController {
 public:
  virtual ~LocalController() = default;
  virtual Twist2d Compute(const Path& path, const Pose2d& pose, Twist2d current,
                           const LayeredCostmap& costmap,
                           const std::vector<PredictedObstacle>& dynamic_obstacles = {}) const = 0;
  virtual bool CollisionImminent(const Pose2d& pose, Twist2d command,
                                 const LayeredCostmap& costmap) const = 0;
};

}  // namespace navigation2d
