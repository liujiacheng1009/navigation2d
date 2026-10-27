#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "navigation2d/costmap/layered_costmap.h"
#include "navigation2d/execution/algorithm_budget.h"
#include "navigation2d/planning/path_metadata.h"
#include "navigation2d/geometry/path_2d.h"

namespace navigation2d {

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

enum class GlobalPlanStatus {
  kSuccess,
  kNoPath,
  kStartOccupied,
  kGoalOccupied,
  kCancelled,
  kBudgetExceeded,
  kInvalidInput,
  kInvalidOutput,
  kBackendError,
  kNotRun
};
struct GlobalPlanRequest {
  const LayeredCostmap& costmap;
  const Pose2d& start;
  const Pose2d& goal;
  std::string_view frame_id = "navigation_map";
  AlgorithmBudget budget{};
};
struct GlobalPlanResult {
  GlobalPlanStatus status = GlobalPlanStatus::kNoPath;
  Path path;
  PlannedPathMetadata metadata;
  std::string reason;
  [[nodiscard]] bool succeeded() const { return status == GlobalPlanStatus::kSuccess; }
};

class GlobalPlanner {
 public:
  virtual ~GlobalPlanner() = default;
  virtual Path Plan(const LayeredCostmap& costmap, const Pose2d& start,
                    const Pose2d& goal) const;
  // Borrowed map/poses must remain stable for the whole synchronous call.
  // New backends may override PlanRequest to poll budgets inside search loops. The
  // compatibility adapter preserves legacy error text and rejects late/invalid output.
  GlobalPlanResult PlanWithContext(const GlobalPlanRequest& request) const;
  virtual PathHeadingSemantics HeadingSemantics() const noexcept {
    return PathHeadingSemantics::kUnspecified;
  }
  virtual GlobalPlanningDiagnostics Diagnostics() const { return {}; }

 protected:
  // Override the request hook for typed outcomes/cooperative search. Public
  // PlanWithContext always validates budgets, frame/revision and finite output.
  virtual GlobalPlanResult PlanRequest(const GlobalPlanRequest& request) const;
};

}  // namespace navigation2d
