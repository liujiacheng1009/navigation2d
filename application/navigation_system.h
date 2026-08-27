#pragma once

#include <string>
#include "navigation2/types.h"

namespace navigation2d {

struct RunResult {
  std::string status = "FAILED";
  double duration_s = 0.;
  double goal_error_m = 0.;
  int collisions = 0;
  int steps = 0;
  Path trajectory;
};

class NavigationSystem {
 public:
  RunResult Run(const std::string& world_path, Pose2d start, Pose2d goal) const;
};

}  // namespace navigation2d
