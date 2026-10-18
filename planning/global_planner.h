#pragma once

#include <cstddef>
#include <vector>

#include "navigation2d/costmap/layered_costmap.h"
#include "navigation2d/geometry/pose_2d.h"

namespace navigation2d {

using Path = std::vector<Pose2d>;

struct GlobalPlanningDiagnostics {
  std::size_t expansions = 0;
  std::size_t generated = 0;
  double elapsed_s = 0.;
  double first_solution_s = 0.;
  double suboptimality_bound = 1.;
  bool obstacle_heuristic_cache_hit = false;
  bool incremental_reuse = false;
  std::size_t repaired_states = 0;
  bool fallback_used = false;
  double path_min_clearance_m = 0.;
  double path_max_curvature = 0.;
};

class GlobalPlanner {
 public:
  virtual ~GlobalPlanner() = default;
  virtual Path Plan(const LayeredCostmap& costmap, const Pose2d& start,
                    const Pose2d& goal) const = 0;
  virtual GlobalPlanningDiagnostics Diagnostics() const { return {}; }
};

}  // namespace navigation2d
