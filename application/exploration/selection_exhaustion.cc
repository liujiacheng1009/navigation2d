#include "navigation2d/application/exploration/selection_exhaustion.h"

#include "navigation2d/application/exploration/completion_execution.h"
#include "navigation2d/application/exploration/tour_execution.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
void SelectionExhaustion::Resolve(FrontierExplorer& explorer, CompletionExecution& completion,
                                  TourExecution& tour, ExhaustedSelection observation,
                                  SelectionExhaustionActions& actions) {
  actions.StopSelectionMotion();
  if (!tour.goals().empty() || observation.attempted_plans > 0 ||
      explorer.raw_frontier_cells() > 0) {
    ResolveBlocked(explorer, completion, tour, observation, actions);
    return;
  }
  ++empty_cycles_;
  blocked_cycles_ = 0;
  // A mapper can briefly publish a small, fully-known crop before the next
  // scan expands it.  Never report a zero-goal mission as complete from
  // that bootstrap artifact: wait for enough observed support unless an
  // actual frontier visit has already occurred.
  constexpr std::size_t kMinimumInitialCoverageCells = 25000;
  const bool map_is_mature =
      explorer.KnownCells() >= kMinimumInitialCoverageCells || explorer.completed_goals() > 0;
  actions.SelectionStatus(map_is_mature ? "no reachable frontier; confirming completion"
                                        : "initial map still growing; waiting for frontiers");
  if (map_is_mature && empty_cycles_ >= 12) {
    actions.BeginReturn(false);
  }
}
void SelectionExhaustion::ResolveBlocked(FrontierExplorer& explorer,
                                         CompletionExecution& completion, TourExecution& tour,
                                         ExhaustedSelection observation,
                                         SelectionExhaustionActions& actions) {
  const std::size_t known_now = explorer.KnownCells();
  // One newly marked cell is scan noise, not map progress.  Resetting the
  // no-executable budget on +1 kept warehouse leftovers spinning until
  // the mission timeout without ever starting the home path.
  constexpr std::size_t kMeaningfulKnownGrowth = 500;
  if (known_now >= last_known_cells_ + kMeaningfulKnownGrowth) {
    blocked_cycles_ = 0;
    last_known_cells_ = known_now;
    // New map evidence may open a tip the previous leftover missed.
    completion.ResetBudget();
  }
  actions.NoteKnownProgress(known_now);
  ++blocked_cycles_;
  const std::size_t reachable_unknown =
      explorer.ReachableUnknownCells(observation.robot.x, observation.robot.y);
  // Nominated mouths that all fail StartNavigation are not executable
  // either. Requiring candidates==0 skipped leftover completion and
  // returned PARTIAL while the same 3 unplannable T-labels looped.
  // Empty tour with candidates>0 is the BLOCKED-DFS trap: mouths exist but
  // Order wiped them — treat as no-executable so completing/entry runs.
  const bool tour_unplannable = tour.goals().empty() && observation.attempted_plans > 0;
  const bool tour_empty_with_candidates = tour.goals().empty() &&
                                          observation.attempted_plans == 0 &&
                                          explorer.candidate_frontier_goals() > 0;
  const bool no_executable_mouths =
      explorer.candidate_frontier_goals() == 0 || tour_unplannable || tour_empty_with_candidates;
  // Stay for a door only when this room still has a goal to run.
  // Zero candidates means the room is finished; re-entering it here
  // put the stack back and skipped coverage.
  if (explorer.candidate_frontier_goals() > 0 && explorer.ActiveHoldsPassage()) {
    actions.QueueActiveRegionFinish();
    if (!tour.goals().empty()) {
      actions.DeferBlockedSelection("door still open; staying in the active room");
      return;
    }
  }
  if (no_executable_mouths && explorer.ShouldCoverResiduals()) {
    if (completion.CanProbe()) {
      actions.StartCompletingProbe();
      return;
    }
    // Completing probe can fail while sealed mouths still open into
    // unknown rooms on later map updates. Returning here left rooms
    // unexplored with PARTIAL and unresolved frontiers.
    if (actions.TipPinchedNow() &&
        actions.TryRescueDistantMouths("no-executable while tip-pinched")) {
      return;
    }
    if (actions.ShouldKeepWaitingForFrontiers()) {
      actions.SelectionStatus("openable unknown remains; waiting for map update");
      return;
    }
    actions.BeginReturnOrRescue(reachable_unknown > 0 || explorer.raw_frontier_cells() > 0);
    return;
  }
  WaitForMap(explorer, completion, observation.attempted_plans, actions);
}
void SelectionExhaustion::WaitForMap(FrontierExplorer& explorer, CompletionExecution& completion,
                                     int attempted_plans, SelectionExhaustionActions& actions) {
  // Failed mouths were already RecordAttempt'd. Wait for the next map
  // tick instead of a busy reject/rebuild loop (that was the hesitation).
  actions.ClearBlockedSelection();
  actions.SelectionStatus("frontiers locally blocked; waiting for map update");
  // Tip / open aisle: do not sit through a long empty budget — peel and
  // rescue distant mouths (kitchen) before PARTIAL.
  const int wait_budget =
      actions.TipPinchedNow()
          ? 2
          : ((attempted_plans > 0 && actions.SelectionFrontSwingClear()) ? 3 : 6);
  if (blocked_cycles_ >= wait_budget) {
    if (actions.TipPinchedNow() &&
        actions.TryRescueDistantMouths("wait-budget tip; rescue distant")) {
      return;
    }
    if (completion.CanProbe() && explorer.ShouldCoverResiduals()) {
      actions.StartCompletingProbe();
      return;
    }
    if (actions.ShouldKeepWaitingForFrontiers()) {
      return;
    }
    actions.BeginReturnOrRescue(true);
  }
}
}  // namespace navigation2d::exploration
