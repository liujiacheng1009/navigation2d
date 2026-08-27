#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include "navigation2d/application/navigation_config.h"
#include "navigation2d/types.h"

namespace navigation2d::simulation {

struct ObstacleEvent { double appear_s; double disappear_s; double x; double y; };
struct RunResult {
  std::string status = "FAILED";
  double duration_s = 0.;
  double goal_error_m = 0.;
  double goal_heading_error_rad = 0.;
  int collisions = 0;
  int steps = 0;
  int replans = 0;
  int emergency_stops = 0;
  int recoveries = 0;
  double global_path_length_m = 0.;
  std::uint64_t costmap_digest = 0;
  Path trajectory;
};

class NavigationSimulator {
 public:
  explicit NavigationSimulator(NavigationConfig config) : config_(config) {}
  RunResult Run(const std::string& world_path, Pose2d start, Pose2d goal,
                const std::vector<ObstacleEvent>& obstacles = {}) const;
 private:
  NavigationConfig config_;
};

}  // namespace navigation2d::simulation
