#include "navigation2d/application/navigation_failures.h"

#include <cassert>

#include "navigation2d/application/execution_watchdog.h"
#include "navigation2d/control/local_controller.h"
using namespace navigation2d;
int main() {
  NavigationState state;
  const auto pose = MakePose2d(1., 2., 0.);
  Path path{pose};
  std::optional<Pose2d> goal = pose;
  ExecutionWatchdog execution({.1, .1, 5., 2});
  double next_replan = 3.;
  NavigationFailures failures(state, path, goal, execution, next_replan, 2);
  state.command = {1., 1.};
  state.path_signature = 42;
  failures.RejectPlan(GlobalPlanStatus::kNoPath, "no route");
  assert(path.empty() && state.command.linear == 0. && state.path_signature == 0);
  assert(goal && state.status == NavigationStatus::kNavigating && !failures.latched());
  failures.RejectPlan(GlobalPlanStatus::kNoPath, "no route");
  assert(state.status == NavigationStatus::kNavigating);
  failures.RejectPlan(GlobalPlanStatus::kNoPath, "no route");
  assert(state.status == NavigationStatus::kBlocked && !failures.latched());
  failures.ResetPlanningBudget();
  failures.RejectPlan(GlobalPlanStatus::kNoPath, "cooled", RouteRejection::kSignatureExcluded);
  assert(state.status == NavigationStatus::kNavigating);
  assert(state.route_rejection == RouteRejection::kSignatureExcluded);

  failures.RejectPlan(GlobalPlanStatus::kInvalidOutput, "invalid route");
  assert(failures.latched() && goal && state.status == NavigationStatus::kBlocked);
  failures.ResetPlanningBudget();
  assert(failures.latched());  // Route budget reset cannot clear a backend fault.
  failures.Stop(AlgorithmStop::kBudgetExceeded);
  assert(goal && failures.latched() && state.status == NavigationStatus::kBlocked);
  failures.Stop(AlgorithmStop::kCancelled);
  assert(!goal && path.empty() && !failures.latched() && state.status == NavigationStatus::kIdle);

  goal = pose;
  LocalControlResult invalid;
  invalid.status = LocalControlStatus::kInvalidOutput;
  state.execution_failure = NavigationFailure::kControllerNoMotion;
  failures.RejectControl(invalid);
  assert(failures.latched() && state.status == NavigationStatus::kBlocked);
  assert(state.execution_failure == NavigationFailure::kNone);
  assert(state.controller_diagnostics.status == ControllerSolveStatus::kFailure);
  assert(state.planning_failure_reason == "controller produced no valid command");
  failures.ClearFault();
  invalid.status = LocalControlStatus::kBudgetExceeded;
  invalid.reason = "solver budget";
  failures.RejectControl(invalid);
  assert(goal && !failures.latched() && state.status == NavigationStatus::kNavigating);
  assert(state.algorithm_stop == AlgorithmStop::kBudgetExceeded);
  assert(state.planning_failure_reason == "solver budget");
  invalid.status = LocalControlStatus::kCancelled;
  invalid.reason = "request cancelled";
  failures.RejectControl(invalid);
  assert(!goal && state.status == NavigationStatus::kIdle);
  assert(state.planning_failure_reason == "request cancelled");

  path = {pose};
  state.status = NavigationStatus::kNavigating;
  const auto recoveries = state.recoveries;
  assert(!failures.Apply(NavigationFailure::kNone));
  assert(failures.Apply(NavigationFailure::kProgressExhausted));
  assert(path.size() == 1 && state.status == NavigationStatus::kBlocked);
  assert(state.recoveries == recoveries + 1 && !failures.latched());
  state.status = NavigationStatus::kNavigating;
  assert(failures.Apply(NavigationFailure::kProgressRetry));
  assert(path.empty() && next_replan == 0. && state.recoveries == recoveries + 2);
  assert(state.status == NavigationStatus::kNavigating);
  goal = pose;
  failures.RejectPlan(GlobalPlanStatus::kCancelled, "planner cancel");
  assert(!goal && state.status == NavigationStatus::kIdle);
  assert(state.planning_status == GlobalPlanStatus::kCancelled);
}
