#include "navigation2d/application/exploration/goal_failure_execution.h"

#include "navigation2d/application/exploration/exploration_mission.h"
#include "navigation2d/application/exploration/return_coordinator.h"
#include "navigation2d/application/navigation_failure.h"
#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
bool GoalFailureExecution::Returning() const {
  return mission_.phase() == MissionPhase::kReturning || returning_.returning();
}
bool GoalFailureExecution::HandleMonitorStop(const NavigationState& state, double front,
                                             double side, const GoalExecutionClock& clock,
                                             GoalFailureActions& actions) {
  const bool monitor_stop = state.status == NavigationStatus::kBlocked &&
                            state.collision_monitor_action == CollisionMonitorAction::kStop;
  if (mission_.phase() != MissionPhase::kExploring || returning_.returning() || !monitor_stop) {
    return false;
  }
  const double clearance = actions.ActivePathMargin(state.global_planning.path_min_clearance_m);
  if (actions.IsCornerTipStall(clearance, front, side) && watch_.ProgressOlderThan(2., clock)) {
    actions.BreakWallScrapeThrash("corner-tip CARROT stall");
    return true;
  }
  if (AxisSwingClear(front)) {
    ExecuteOpenForwardStop(entry_, explorer_, recovery_, watch_, clock,
                           {state.global_planning.path_min_clearance_m, side}, actions);
    return true;
  }
  if (HitInsideBody(front)) {
    ExecuteForwardBlocked(entry_, explorer_, tour_, recovery_, retreat_, actions.ActiveGoal(),
                          actions);
    return true;
  }
  return false;
}
bool GoalFailureExecution::HandleSegmentStall(const NavigationState& state, bool first_segment,
                                              double front, double side,
                                              GoalFailureActions& actions) {
  const bool force_return = first_segment ? actions.NoteFirstSegmentFailure(state) : false;
  if (Returning()) {
    if (actions.TryReturnSegmentFallback()) {
      return true;
    }
  } else {
    if (entry_.locked() || entry_.pending()) {
      if (!entry_.pending()) {
        entry_.SetPending(entry_.locked());
      }
      retreat_.AllowRetry();
      actions.CancelRecoveryNavigation();
      // A real no-motion failure uses bounded open-loop crawl, rather than a
      // replan storm. Plugin diagnostic text cannot impersonate that failure.
      if (IsNoMeasuredMotion(state.execution_failure) && actions.FailurePoseReady()) {
        actions.PrepareNoMotionEntry(state);
        if (entry_.crawl_ticks() == 0) {
          entry_.StartCrawl();
        }
        return true;
      }
      ResolveLockedEntryStall(entry_, explorer_, recovery_, actions);
      return true;
    }
    // Preserve scrape budgets: clearing here caused endless first-scrape loops.
    explorer_.RecordAttempt(actions.ActiveGoal(), false);
    actions.DemoteViewpointForSeconds(actions.ActiveGoal(), 25.);
  }
  if (!Returning() &&
      actions.IsPinchGeometryStall(
          actions.ActivePathMargin(state.global_planning.path_min_clearance_m), front, side)) {
    actions.BreakWallScrapeThrash("room-side / doorway pinch stall");
    return true;
  }
  if (force_return && !returning_.returning()) {
    actions.CancelRecoveryNavigation();
    tour_.Suspend();
    actions.BeginReturn(true);
    return true;
  }
  return false;
}
void GoalFailureExecution::FinishFailure(const NavigationState& state, bool first_segment,
                                         bool mid_path, GoalFailureActions& actions) {
  if (!mid_path && !IsNoMeasuredMotion(state.execution_failure) &&
      actions.ReplanActiveGoalOnLatestMap(state.status == NavigationStatus::kBlocked
                                              ? "controller could not execute validated route"
                                              : "goal execution timed out")) {
    return;
  }
  if (IsNoMeasuredMotion(state.execution_failure) && !Returning()) {
    actions.CancelRecoveryNavigation();
    explorer_.RecordAttempt(actions.ActiveGoal(), false);
    actions.DemoteViewpointForSeconds(actions.ActiveGoal(), 25.);
    if (!tour_.goals().empty()) {
      tour_.Advance();
    }
    if (actions.TryStartEscape()) {
      return;
    }
    actions.SelectAfterEntry();
    actions.PublishStatus("SELECTING", "no motion against the jamb; leaving this route");
    return;
  }
  actions.CancelRecoveryNavigation();
  recovery_.Clear();
  if (Returning()) {
    if (actions.StartBreadcrumbTowardHome()) {
      return;
    }
    returning_.MarkPartial();
    actions.Finish(true, "exploration ended; return path blocked on online map");
  } else if (mission_.phase() == MissionPhase::kCompleting) {
    if (!first_segment && !mid_path) {
      explorer_.RecordAttempt(actions.ActiveGoal(), false);
    }
    actions.FinishCompleting();
  } else {
    if (!tour_.goals().empty()) {
      tour_.Advance();
    }
    actions.SelectAfterEntry();
    actions.PublishStatus(
        "SELECTING", first_segment ? "first-segment failed; skipping cooled viewpoint"
                                   : (mid_path ? "mid-path pinch stall; skipping viewpoint"
                                               : "frontier failed; continuing committed tour"));
  }
}
void GoalFailureExecution::Resolve(const NavigationState& state, double front, double side,
                                   const GoalExecutionClock& clock, GoalFailureActions& actions) {
  const bool exploring = mission_.phase() == MissionPhase::kExploring && !returning_.returning();
  if (HandleMonitorStop(state, front, side, clock, actions)) {
    return;
  }
  actions.ReportExecutionFailure(state);
  const bool first_segment = state.path_progress_m < .05;
  const bool pinch = HitBlocksSwing(front) && HitBlocksSwing(side);
  const bool mid_path =
      state.path_progress_m >= .05 && watch_.ProgressOlderThan(pinch ? 5. : 10., clock);
  const bool timed_out = watch_.TimedOut(clock);
  if (exploring && timed_out && !first_segment && !mid_path) {
    actions.DemoteActiveGoal("goal timeout; demoting viewpoint");
    return;
  }
  if ((first_segment || mid_path) &&
      HandleSegmentStall(state, first_segment, front, side, actions)) {
    return;
  }
  FinishFailure(state, first_segment, mid_path, actions);
}
}  // namespace navigation2d::exploration
