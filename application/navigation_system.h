#pragma once

#include <string>
#include <cstdint>
#include <vector>
#include "navigation2/application/navigation_config.h"
#include "navigation2/types.h"

namespace navigation2d {

struct ObstacleEvent { double appear_s; double disappear_s; double x; double y; };
struct RunResult {
  std::string status = "FAILED";
  double duration_s = 0.;
  double goal_error_m = 0.;
  int collisions = 0;
  int steps = 0;
  int replans = 0;
  int emergency_stops = 0;
  int recoveries = 0;
  std::uint64_t costmap_digest = 0;
  Path trajectory;
};

class NavigationSystem {
 public:
  explicit NavigationSystem(NavigationConfig config = {}) : config_(config) {}
  RunResult Run(const std::string& world_path, Pose2d start, Pose2d goal,
                const std::vector<ObstacleEvent>& obstacles = {}) const;

 private:
  NavigationConfig config_;
};

}  // namespace navigation2d
