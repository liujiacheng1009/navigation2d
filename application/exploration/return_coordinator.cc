#include "navigation2d/application/exploration/return_coordinator.h"

#include <cmath>
#include "navigation2d/application/exploration/return_mission_actions.h"
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

void ReturnCoordinator::BeginMission(bool force_partial, ReturnMissionActions& actions) {
  if (actions.TryVisitUnenteredSpace()) {
    return;
  }
  actions.CommitReturnPhase();
  returning_ = true;
  actions.SuspendExplorationForReturn();
  if (force_partial) {
    MarkPartial();
    actions.ReportReturnStart(ReturnStartNotice::kForcedPartial);
  } else if (!actions.ReturnCoverageComplete()) {
    actions.ReportReturnStart(ReturnStartNotice::kCoverageIncomplete);
  }
  actions.ClearReturnRouteCooling();
  UseBreadcrumbs(false);
  const auto result = Execute(actions);
  if (result == ReturnExecutionResult::kEscaping) {
    actions.ResumeReturnEscape(false);
  } else if (result == ReturnExecutionResult::kUnavailable) {
    actions.Finish(false, "return path unavailable; stopping to protect localization");
  }
}
bool ReturnCoordinator::FollowTrail(std::span<const std::pair<double, double>> trail,
                                    const ExplorationGoal& home, double rx, double ry,
                                    const ExplorationGoal& active_goal,
                                    ReturnMissionActions& actions) {
  const double home_distance = std::hypot(rx - home.x, ry - home.y);
  const auto next = NextStep(trail, home, rx, ry, active_goal);
  if (next.kind == ReturnStepKind::kRepeated) {
    return false;
  }
  if (next.kind == ReturnStepKind::kDirect) {
    UseBreadcrumbs(false);
    actions.PrepareReturnEvidence(true);
    return actions.NavigateReturnGoal(home);
  }
  actions.CommitReturnPhase();
  returning_ = true;
  UseBreadcrumbs(true);
  actions.ClearReturnRouteCooling();
  actions.PrepareReturnEvidence(true);
  if (!actions.NavigateReturnGoal(next.goal)) {
    return false;
  }
  actions.ReportBreadcrumbHop(next.goal, rx, ry, home_distance);
  return true;
}
void ReturnCoordinator::RecoverMission(ReturnRecoveryMode mode, const char* unavailable_message,
                                       ReturnMissionActions& actions) {
  const auto result = Recover(mode, actions);
  if (result == ReturnExecutionResult::kEscaping) {
    actions.ResumeReturnEscape(false);
  } else if (result == ReturnExecutionResult::kUnavailable) {
    actions.Finish(true, unavailable_message);
  }
}
void ReturnCoordinator::TickIdle(bool escape_pending, ReturnMissionActions& actions) {
  const auto result = ContinueIdle(escape_pending, actions);
  if (result == ReturnIdleResult::kEscaping) {
    actions.ResumeReturnEscape(true);
  } else if (result == ReturnIdleResult::kUnavailable) {
    actions.Finish(true, "return path unavailable; stopping to protect localization");
  } else if (result == ReturnIdleResult::kRestart) {
    actions.BeginReturn(partial());
  }
}
}  // namespace navigation2d::exploration
