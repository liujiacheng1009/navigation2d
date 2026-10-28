#include "navigation2d/application/exploration/viewpoint_selection.h"

#include <cmath>

#include "navigation2d/application/exploration/completion_execution.h"
#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/entry_recovery_execution.h"
#include "navigation2d/application/exploration/tour_execution.h"
#include "navigation2d/application/exploration/viewpoint_rejections.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
namespace {
using Goal = ExplorationGoal;
double NormalizeAngle(double value) { return std::atan2(std::sin(value), std::cos(value)); }
}  // namespace
void ViewpointSelection::Select(TourPose robot, ViewpointSelectionActions& actions) {
  // Drop mouths the lidar already cleared; keep the remaining committed
  // order. Rebuilding the tour on every SelectNextGoal reordered T1..Tn and
  // made the robot U-turn between alcoves (triangular stutter in open space).
  tour_.SkipResolved(explorer_);
  if (tour_.goals().empty()) {
    actions.BuildTour();
  }
  // A frontier extractor can emit many viewpoints around one wall.  Do not
  // synchronously exhaust that list when the local controller rejects a
  // viewpoint: that turns one transient costmap disagreement into hundreds
  // of failures before the mapper has published its next observation.  This
  // is the same bounded retry principle used by Nav2 recovery trees: try a
  // small, diverse budget, then yield to new sensor/map evidence.
  constexpr int kCandidatePlanBudget = 3;
  int attempted = 0;
  int rotated_reverse = 0;
  const double yaw = robot.yaw;
  while (!tour_.goals().empty() && attempted < kCandidatePlanBudget) {
    const Goal goal = tour_.goals().front();
    if (!explorer_.GoalRegionStillFrontier(goal)) {
      actions.ReportSelectionSkip(SelectionSkip::kResolved, goal);
      tour_.Advance();
      continue;
    }
    if (actions.RecentlyDemoted(goal)) {
      actions.ReportSelectionSkip(SelectionSkip::kDemoted, goal);
      tour_.Advance();
      // Rebuild inside the open room. Do not pull a global goal while a
      // door in this room is still holding the tour.
      if (tour_.goals().empty()) {
        actions.BuildTour();
      }
      continue;
    }
    // Defer reverse goals only when they are far. A tip inside the local
    // pocket must be finished even if it sits behind the robot — otherwise
    // we flee the alcove with a red frontier arc still visible.
    const double bearing = std::atan2(goal.y - robot.y, goal.x - robot.x);
    const double turn = std::abs(NormalizeAngle(bearing - yaw));
    const double goal_range = std::hypot(goal.x - robot.x, goal.y - robot.y);
    if (turn > 1.75 && goal_range > 2.5 &&
        rotated_reverse < static_cast<int>(tour_.goals().size())) {
      tour_.DeferCurrent(goal);
      ++rotated_reverse;
      continue;
    }
    if (actions.NavigateViewpoint(goal)) {
      exhaustion_.GoalAccepted();
      reject_streak_ = 0;
      actions.ViewpointAccepted();
      completion_.ResetBudget();
      return;
    }
    tour_.Advance();
    ++attempted;
    // Start pinched in wall-corner inflation: every other goal fails the same
    // start check. Do not entry-probe / rebuild-tour — leave the pinch first.
    const auto failure = actions.LastViewpointFailure();
    const bool start_pinched = failure.start_occupied;
    if (start_pinched) {
      ++reject_streak_;
      actions.PreparePinchedViewpoint(failure.reason);
      if (actions.EscapeOrShortRetreat("start pinched; peel out of inflation",
                                       "start pinched; short trail retreat")) {
        actions.ClearSelectionRouteCooling();
        return;
      }
      break;
    }
    // Entry only for real corridor mouths. Open-wall / residual frontiers
    // (user: 没有门缝) must not LockEntryMouth — that burned attempts on
    // gap=1.65 open spans and looked like "stuck at a door".
    if (EntryCoordinator::Passable(goal, explorer_)) {
      if (entry_.locked() && !EntryCoordinator::SameMouth(*entry_.locked(), goal)) {
        explorer_.RecordAttempt(goal, false);
        continue;
      }
      actions.LockEntryMouth(goal);
      if (actions.StartEntryProbe()) {
        return;
      }
      if (entry_.attempts() >= entry_.AttemptBudget(explorer_)) {
        actions.AbandonLockedEntryMouth("hallway reject; entry probe exhausted, leaving mouth");
        continue;
      }
      entry_.ClearPending();  // lock retained
      actions.DeferViewpointRetry();
      return;
    }
    navigation2d::exploration::RejectOpenWallViewpoint(
        explorer_, tour_, viewpoint_rejections_, {goal, failure.reason, failure.path_signature},
        actions);
    continue;
  }
  // Same pose keeps rejecting every queued frontier (often cooled first-
  // segment signatures while pinched in an alcove). Retreat along the
  // driven trail before spinning on identical rejections until PARTIAL.
  if (reject_streak_ >= 2 && actions.EscapeOrShortRetreat("reject streak; peel before reselect",
                                                          "reject streak; short trail retreat")) {
    actions.ClearSelectionRouteCooling();
    return;
  }
  exhaustion_.Resolve(explorer_, completion_, tour_, {robot, attempted}, actions);
}
void ViewpointSelection::AfterRetreat(EntryRecoveryActions& actions) {
  ResetRejected();
  if (ResumeEntryAfterRetreat(entry_, explorer_, actions)) {
    return;
  }
  actions.SelectAfterEntry();
  actions.PublishStatus("SELECTING", "left the pinched corridor");
}
}  // namespace navigation2d::exploration
