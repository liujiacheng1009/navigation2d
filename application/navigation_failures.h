#pragma once
#include <optional>
#include <string>

#include "navigation2d/application/navigation_failure.h"
#include "navigation2d/application/navigation_state.h"
#include "navigation2d/geometry/path_2d.h"
namespace navigation2d {
class ExecutionWatchdog;
struct LocalControlResult;
// Owns the algorithm-fault latch and consecutive planning-failure budget.
// Borrows stable session/result objects from NavigationSystem::Impl; all calls
// are serialized. It performs no planning, control, I/O or clock reads.
class NavigationFailures {
 public:
  NavigationFailures(NavigationState& state, Path& path, std::optional<Pose2d>& goal,
                     ExecutionWatchdog& execution, double& next_replan_time,
                     int max_recovery_attempts)
      : state_(state),
        path_(path),
        goal_(goal),
        execution_(execution),
        next_replan_time_(next_replan_time),
        max_recovery_attempts_(max_recovery_attempts) {}
  NavigationFailures(const NavigationFailures&) = delete;
  NavigationFailures& operator=(const NavigationFailures&) = delete;
  bool latched() const { return algorithm_fault_latched_; }
  // A successful route clears only the planning budget. A new goal separately
  // clears the fault latch at its original commit boundary.
  void ClearFault() { algorithm_fault_latched_ = false; }
  void ResetPlanningBudget() { planning_failures_ = 0; }
  void Stop(AlgorithmStop);
  void RejectPlan(GlobalPlanStatus, const std::string& reason,
                  RouteRejection = RouteRejection::kNone);
  // Only for a backend result with has_command()==false.
  void RejectControl(const LocalControlResult&);
  bool Apply(NavigationFailure);

 private:
  NavigationState& state_;
  Path& path_;
  std::optional<Pose2d>& goal_;
  ExecutionWatchdog& execution_;
  double& next_replan_time_;
  int max_recovery_attempts_;
  bool algorithm_fault_latched_ = false;
  int planning_failures_ = 0;
};
}  // namespace navigation2d
