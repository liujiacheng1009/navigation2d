#include "navigation2d/application/exploration/completion_execution.h"

#include <cmath>
#include <utility>

#include "navigation2d/application/exploration/mission_lifecycle.h"
#include "navigation2d/application/exploration/tour_execution.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
namespace {
enum class CandidateOutcome { kContinue, kHandled };
CandidateOutcome HandleCandidate(FrontierExplorer& explorer, const ExplorationGoal& leftover,
                                 const TourPose& robot, CompletionActions& actions) {
  const double near = std::hypot(leftover.x - robot.x, leftover.y - robot.y);
  if (near < .60 && !explorer.GoalInActiveRegion(leftover)) {
    actions.ReportCompletion({.kind = CompletionNoticeKind::kThresholdCell, .goal = leftover});
    return CandidateOutcome::kContinue;
  }
  if (actions.NavigateCompletionGoal(leftover)) {
    return CandidateOutcome::kHandled;
  }
  if (actions.BreakMappedObstacleThrash()) {
    return CandidateOutcome::kHandled;
  }
  actions.LockEntryMouth(leftover);
  if (actions.StartEntryProbe()) {
    actions.ReportCompletion({.kind = CompletionNoticeKind::kEntryProbe, .goal = leftover});
    return CandidateOutcome::kHandled;
  }
  actions.ReportCompletion({.kind = CompletionNoticeKind::kUnplannable, .goal = leftover});
  explorer.RecordAttempt(leftover, false);
  actions.ClearEntryMouthLock();
  return CandidateOutcome::kContinue;
}
}  // namespace
void CompletionExecution::Start(FrontierExplorer& explorer, MissionLifecycle& mission,
                                TourExecution& tour, TourPose robot, CompletionActions& actions) {
  ++rounds_;
  mission.BeginCompleting();
  tour.Suspend();
  // Retry residual mouths hidden by temporary exploring failures.
  explorer.ClearFailedBlacklist();
  actions.ClearCompletionRouteCooling();
  auto leftovers = explorer.SelectLeftoverApproaches(robot.x, robot.y, 3);
  leftovers = explorer.OrderByRegion(std::move(leftovers), robot.x, robot.y, robot.yaw);
  if (leftovers.empty()) {
    actions.ReportCompletion({.kind = CompletionNoticeKind::kEmpty,
                              .goal = {},
                              .raw_cells = explorer.raw_frontier_cells(),
                              .rounds = rounds_});
    mission.ResumeExploring();
    if (actions.TipPinchedNow() &&
        actions.TryRescueDistantMouths("completing empty while tip-pinched")) {
      return;
    }
    if (actions.ShouldKeepWaitingForFrontiers()) {
      actions.CompletionStatus("unresolved boundary; waiting for map before return");
      return;
    }
    ReturnOrRescue(explorer.raw_frontier_cells() > 0, actions);
    return;
  }
  for (const auto& leftover : leftovers) {
    if (HandleCandidate(explorer, leftover, robot, actions) == CandidateOutcome::kHandled) {
      return;
    }
  }
  // One failed leftover does not abandon unfinished rooms.
  mission.ResumeExploring();
  if (actions.TipPinchedNow() &&
      actions.TryRescueDistantMouths("leftovers unplannable while tip-pinched")) {
    return;
  }
  if (actions.ShouldKeepWaitingForFrontiers()) {
    actions.CompletionStatus("leftover blocked; waiting for map update before return");
    return;
  }
  ReturnOrRescue(explorer.raw_frontier_cells() > 0 ||
                              explorer.ReachableUnknownCells(robot.x, robot.y) > 0, actions);
}
void CompletionExecution::Finish(FrontierExplorer& explorer, MissionLifecycle& mission,
                                 TourExecution& tour, CompletionActions& actions) {
  mission.ResumeExploring();
  tour.Suspend();
  actions.BuildTour();
  if (!tour.goals().empty() || explorer.candidate_frontier_goals() > 0) {
    actions.ReportCompletion({.kind = CompletionNoticeKind::kCandidatesReady,
                              .goal = {},
                              .candidates = explorer.candidate_frontier_goals(),
                              .tour_size = tour.goals().size()});
    actions.SelectNextGoal();
    return;
  }
  if (explorer.raw_frontier_cells() > 0 && CanProbe()) {
    // The adapter takes a fresh pose snapshot at the original call boundary.
    // Start is exposed separately because the finish operation has no pose.
    actions.StartCompletingProbe();
    return;
  }
  if (actions.ShouldKeepWaitingForFrontiers()) {
    actions.CompletionStatus("completing emptied tour; waiting for unresolved frontiers");
    return;
  }
  ReturnOrRescue(explorer.raw_frontier_cells() > 0, actions);
}
void CompletionExecution::ReturnOrRescue(bool partial, CompletionActions& actions) {
  if (partial && actions.TryRescueDistantMouths("before PARTIAL return")) {
    return;
  }
  actions.BeginReturn(partial);
}
}  // namespace navigation2d::exploration
