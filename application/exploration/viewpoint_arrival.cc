#include "navigation2d/application/exploration/viewpoint_arrival.h"

#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/tour_execution.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
namespace {
void AdvanceIfPresent(TourExecution& tour) {
  if (!tour.goals().empty()) {
    tour.Advance();
  }
}
void RetrySelection(ViewpointArrivalActions& actions) {
  actions.DeferViewpointSelection(.35, "at doorway; retrying centered entry probe");
}
}  // namespace
void ViewpointArrival::Near(FrontierExplorer& explorer, EntryCoordinator& entry,
                            TourExecution& tour, const ExplorationGoal& goal, TourPose robot,
                            ViewpointArrivalActions& actions) {
  const bool room_beyond = explorer.UnenteredRoomBeyond(goal);
  if (entry.pending() || explorer.GoalRegionStillFrontier(goal) || room_beyond) {
    if (!entry.pending()) {
      if (room_beyond && !explorer.GoalRegionStillFrontier(goal)) {
        actions.ReportViewpointArrival(ViewpointArrivalNotice::kNearRoomBeyond, goal, 0, 0.);
      }
      actions.LockEntryMouth(goal);
    }
    if (entry.Satisfied(robot.x, robot.y, goal, explorer)) {
      actions.AcknowledgeEntry(EntryArrival::kNearProbe);
      return;
    }
    if (entry.attempts() < entry.AttemptBudget(explorer) && actions.StartEntryProbe()) {
      return;
    }
    // A rejected probe may consume budget: query again after the callback.
    if (entry.attempts() < entry.AttemptBudget(explorer)) {
      RetrySelection(actions);
      return;
    }
    actions.AbandonLockedEntryMouth("at doorway; entry incomplete, leaving mouth");
    return;
  }
  explorer.RecordAttempt(goal, true);
  actions.ClearEntryMouthLock();
  AdvanceIfPresent(tour);
  actions.DeferViewpointSelection(0., "arrived at viewpoint; continuing tour");
}
void ViewpointArrival::Succeeded(FrontierExplorer& explorer, EntryCoordinator& entry,
                                 TourExecution& tour, const ExplorationGoal& goal, TourPose robot,
                                 ViewpointArrivalActions& actions) {
  if (entry.pending()) {
    if (entry.Satisfied(robot.x, robot.y, goal, explorer)) {
      actions.AcknowledgeEntry(EntryArrival::kNavigationSucceeded);
    } else if (entry.attempts() < entry.AttemptBudget(explorer) && actions.StartEntryProbe()) {
      return;
    } else {
      actions.ReportViewpointArrival(ViewpointArrivalNotice::kEntryIncomplete, *entry.pending(),
                                     entry.attempts(), entry.Depth(robot.x, robot.y));
      actions.AbandonLockedEntryMouth("frontier entry blocked; unresolved boundary retained");
    }
  } else if (explorer.UnenteredRoomBeyond(goal) || (explorer.GoalRegionStillFrontier(goal) &&
                                                    EntryCoordinator::Passable(goal, explorer))) {
    if (!explorer.GoalRegionStillFrontier(goal)) {
      actions.ReportViewpointArrival(ViewpointArrivalNotice::kReachedRoomBeyond, goal, 0, 0.);
    }
    actions.LockEntryMouth(goal);
    if (actions.StartEntryProbe()) {
      return;
    }
    if (entry.attempts() < entry.AttemptBudget(explorer)) {
      RetrySelection(actions);
      return;
    }
    actions.AbandonLockedEntryMouth("frontier reached but no safe entry probe; leaving mouth");
  } else if (explorer.GoalRegionStillFrontier(goal)) {
    explorer.RecordAttempt(goal, true);
    AdvanceIfPresent(tour);
    actions.DeferViewpointSelection(0., "open-wall frontier observed; continuing tour");
  } else {
    explorer.RecordAttempt(goal, true);
    AdvanceIfPresent(tour);
    const bool need_finish =
        region_finish_pending_ || (explorer.GoalInActiveRegion(goal) &&
                                   explorer.ReachableUnknownCells(robot.x, robot.y) > 20);
    region_finish_pending_ = false;
    if (need_finish) {
      actions.QueueActiveRegionFinish();
      actions.DeferViewpointSelection(0., "active room unfinished; covering local leftovers");
      return;
    }
    actions.DeferViewpointSelection(0., "frontier reached; continuing active region tour");
  }
}
}  // namespace navigation2d::exploration
