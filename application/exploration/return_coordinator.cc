#include "navigation2d/application/exploration/return_coordinator.h"

#include <cmath>
namespace navigation2d::exploration {
namespace {
constexpr int kMaximumEscapes = 5;
}
bool ReturnCoordinator::StartBoundedEscape(ReturnActions& actions) {
  if (escapes_ >= kMaximumEscapes || !actions.TryStartEscape()) {
    return false;
  }
  CountEscape();
  return true;
}
ReturnExecutionResult ReturnCoordinator::TryTrailOrEscape(ReturnActions& actions) {
  if (actions.StartBreadcrumbTowardHome()) {
    return ReturnExecutionResult::kTracking;
  }
  return StartBoundedEscape(actions) ? ReturnExecutionResult::kEscaping
                                     : ReturnExecutionResult::kUnavailable;
}
ReturnExecutionResult ReturnCoordinator::Execute(ReturnActions& actions) {
  if (actions.PlanHome(ReturnPlanStage::kInitial)) {
    ResetEscapes();
    actions.HomeAccepted(ReturnPlanStage::kInitial);
    return ReturnExecutionResult::kTracking;
  }
  const auto fallback = TryTrailOrEscape(actions);
  if (fallback == ReturnExecutionResult::kTracking) {
    ResetEscapes();
  }
  if (fallback != ReturnExecutionResult::kUnavailable) {
    return fallback;
  }
  if (actions.PlanHome(ReturnPlanStage::kAfterEscapes)) {
    ResetEscapes();
    actions.HomeAccepted(ReturnPlanStage::kAfterEscapes);
    return ReturnExecutionResult::kTracking;
  }
  // Final trail acceptance intentionally retains the escape count.
  return actions.StartBreadcrumbTowardHome() ? ReturnExecutionResult::kTracking
                                             : ReturnExecutionResult::kUnavailable;
}
ReturnExecutionResult ReturnCoordinator::Recover(ReturnRecoveryMode mode, ReturnActions& actions) {
  const auto fallback = TryTrailOrEscape(actions);
  if (fallback != ReturnExecutionResult::kUnavailable) {
    return fallback;
  }
  if (mode == ReturnRecoveryMode::kPathFailure &&
      actions.PlanHome(ReturnPlanStage::kAfterEscapes)) {
    return ReturnExecutionResult::kTracking;
  }
  MarkPartial();
  return ReturnExecutionResult::kUnavailable;
}
ReturnIdleResult ReturnCoordinator::ContinueIdle(bool escape_pending, ReturnActions& actions) {
  if (escape_pending) {
    return ReturnIdleResult::kWaiting;
  }
  if (breadcrumbs_ && actions.StartBreadcrumbTowardHome()) {
    return ReturnIdleResult::kTracking;
  }
  if (StartBoundedEscape(actions)) {
    return ReturnIdleResult::kEscaping;
  }
  if (actions.StartBreadcrumbTowardHome()) {
    return ReturnIdleResult::kTracking;
  }
  if (escapes_ >= kMaximumEscapes) {
    return actions.PlanHome(ReturnPlanStage::kIdleFinal) ? ReturnIdleResult::kTracking
                                                         : ReturnIdleResult::kUnavailable;
  }
  return ReturnIdleResult::kRestart;
}
ExplorationGoal ReturnCoordinator::Home(const ExplorationGrid* map, ExplorationGoal home) const {
  return SnapReturnHome(map, home);
}
ReturnStep ReturnCoordinator::NextStep(std::span<const std::pair<double, double>> trail,
                                       const ExplorationGoal& home, double rx, double ry,
                                       const ExplorationGoal& active_goal) const {
  return SelectReturnStep(trail, home, {rx, ry}, active_goal, breadcrumbs_);
}
void ReturnCoordinator::CompleteArrival(const ExplorationGoal& home, double robot_x, double robot_y,
                                        ReturnArrivalActions& actions) {
  const double home_distance = std::hypot(robot_x - home.x, robot_y - home.y);
  if (breadcrumbs_ && home_distance > .40) {
    if (actions.PlanDirectAfterHop(home)) {
      breadcrumbs_ = false;
      actions.DirectHomeAfterHopAccepted();
      return;
    }
    if (actions.StartBreadcrumbTowardHome()) {
      return;
    }
  }
  breadcrumbs_ = false;
  actions.FinishReturn(partial_);
}

}  // namespace navigation2d::exploration
