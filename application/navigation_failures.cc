#include "navigation2d/application/navigation_failures.h"

#include "navigation2d/application/execution_watchdog.h"
#include "navigation2d/control/local_controller.h"
namespace navigation2d {
void NavigationFailures::Stop(AlgorithmStop stop) {
  state_.algorithm_stop = stop;
  state_.command = {};
  state_.requested_command = {};
  state_.published_command = {};
  state_.controller_commanded_motion = false;
  state_.execution_failure = NavigationFailure::kNone;
  state_.planning_failure_reason =
      stop == AlgorithmStop::kCancelled ? "navigation cancelled" : "navigation budget exceeded";
  if (stop == AlgorithmStop::kCancelled) {
    goal_.reset();
    path_.clear();
    state_.path_signature = 0;
    algorithm_fault_latched_ = false;
    state_.status = NavigationStatus::kIdle;
  } else {
    state_.status = algorithm_fault_latched_
                        ? NavigationStatus::kBlocked
                        : (goal_ ? NavigationStatus::kNavigating : NavigationStatus::kIdle);
  }
}

void NavigationFailures::RejectPlan(GlobalPlanStatus status, const std::string& reason,
                                    RouteRejection rejection) {
  path_.clear();
  state_.command = {};
  state_.path_signature = 0;
  state_.planning_status = status;
  state_.route_rejection = rejection;
  state_.planning_failure_reason = reason;
  state_.execution_failure = NavigationFailure::kNone;
  if (status == GlobalPlanStatus::kCancelled || status == GlobalPlanStatus::kBudgetExceeded) {
    Stop(status == GlobalPlanStatus::kCancelled ? AlgorithmStop::kCancelled
                                                : AlgorithmStop::kBudgetExceeded);
    return;
  }
  if (status == GlobalPlanStatus::kInvalidInput || status == GlobalPlanStatus::kInvalidOutput ||
      status == GlobalPlanStatus::kBackendError) {
    algorithm_fault_latched_ = true;
    state_.published_command = {};
    state_.requested_command = {};
    state_.controller_commanded_motion = false;
    state_.status = NavigationStatus::kBlocked;
    return;
  }
  execution_.PlanningFailed();
  ++planning_failures_;
  state_.status = planning_failures_ > max_recovery_attempts_ ? NavigationStatus::kBlocked
                                                              : NavigationStatus::kNavigating;
}

bool NavigationFailures::Apply(NavigationFailure failure) {
  if (failure == NavigationFailure::kNone) {
    return false;
  }
  const auto effects = EffectsFor(failure);
  state_.execution_failure = failure;
  if (effects.record_recovery) {
    ++state_.recoveries;
  }
  if (effects.clear_route) {
    path_.clear();
  }
  state_.command = {};
  if (effects.blocked) {
    state_.status = NavigationStatus::kBlocked;
  }
  if (effects.reset_replan_timer) {
    next_replan_time_ = 0.;
  }
  if (!effects.message.empty()) {
    state_.planning_failure_reason = effects.message;
  }
  return true;
}

void NavigationFailures::RejectControl(const LocalControlResult& computed) {
  state_.controller_diagnostics = computed.diagnostics.value_or(ControllerDiagnostics{});
  if (!computed.diagnostics) {
    state_.controller_diagnostics.status = ControllerSolveStatus::kFailure;
  }
  state_.command = {};
  state_.requested_command = {};
  state_.published_command = {};
  state_.controller_commanded_motion = false;
  if (computed.status == LocalControlStatus::kCancelled ||
      computed.status == LocalControlStatus::kBudgetExceeded) {
    Stop(computed.status == LocalControlStatus::kCancelled ? AlgorithmStop::kCancelled
                                                           : AlgorithmStop::kBudgetExceeded);
  } else {
    algorithm_fault_latched_ = true;
    state_.status = NavigationStatus::kBlocked;
  }
  state_.execution_failure = NavigationFailure::kNone;
  state_.planning_failure_reason =
      computed.reason.empty() ? "controller produced no valid command" : computed.reason;
}

}  // namespace navigation2d
