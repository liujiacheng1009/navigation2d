#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include "navigation2d/application/navigation_config.h"
#include "navigation2d/geometry/pose_2d.h"
#include "navigation2d/planning/global_planner.h"

namespace navigation2d::simulation {

struct ObstacleEvent {
  double appear_s;
  double disappear_s;
  double x;
  double y;
  double vx = 0.;
  double vy = 0.;
};
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
  std::vector<double> controller_solve_samples_us;
  GlobalPlanningDiagnostics global_planning;
  int obstacle_heuristic_cache_hits = 0;
  int incremental_replans = 0;
  std::size_t incremental_repaired_states = 0;
  std::vector<double> global_plan_samples_s;
  std::vector<double> global_plan_first_solution_samples_s;
  std::size_t global_plan_expansions_total = 0;
  double path_min_clearance_m = 0.;
  double path_max_curvature = 0.;
  std::uint64_t costmap_digest = 0;
  std::vector<Pose2d> trajectory;
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
