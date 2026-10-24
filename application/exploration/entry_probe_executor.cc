#include "navigation2d/application/exploration/entry_probe_executor.h"

#include <cmath>

#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
namespace {
bool Approach(const ExplorationGoal& mouth, EntryProbeActions& actions,
              const EntryProbeObservation& observation) {
  const double rx = observation.robot_x, ry = observation.robot_y;
  const double range = std::hypot(rx - mouth.frontier_x, ry - mouth.frontier_y);
  // Approach along the robot-to-frontier ray. A stored viewpoint can lie
  // behind the robot and already satisfy goal tolerance without crossing.
  const double travel = range - 1.55;
  if (!(travel > kSwingClearance())) {
    return false;
  }
  auto target = mouth;
  target.x = rx + travel * (mouth.frontier_x - rx) / range;
  target.y = ry + travel * (mouth.frontier_y - ry) / range;
  target.yaw = std::atan2(mouth.frontier_y - ry, mouth.frontier_x - rx);
  actions.PrepareEntryPlan(8.);
  if (!actions.NavigateEntry(target)) {
    return false;
  }
  EntryNotice notice{EntryNoticeKind::kApproaching, mouth, target};
  notice.range_m = range;
  actions.ReportEntry(notice);
  return true;
}
bool CloseIfEntered(EntryCoordinator& entry, FrontierExplorer& explorer, RoomVisitLedger& ledger,
                    EntryProbeActions& actions, EntryProbeObservation observation,
                    EntryNotice& notice) {
  const auto& target = notice.target;
  const auto depth = entry.InteriorDepth(observation.robot_x, observation.robot_y, target);
  const auto& mouth = entry.locked().value_or(*entry.pending());
  if (depth && EntryCoordinator::EnoughDepth(
                   *depth, explorer.MouthGapWidth(mouth.frontier_x, mouth.frontier_y))) {
    explorer.RecordAttempt(*entry.pending(), true, false);
    ledger.Forget(*entry.pending());
    // Cool only the mouth, not its region: remaining room cells must stay
    // eligible for the active-room finishing pass.
    actions.CoolEnteredMouth(*entry.pending(), 14.);
    entry.Clear();
    actions.QueueActiveRegionFinish();
    actions.SelectAfterEntry();
    notice.kind = EntryNoticeKind::kEntered;
    notice.robot_x = observation.robot_x;
    notice.robot_y = observation.robot_y;
    actions.ReportEntry(notice);
    return true;
  }
  return false;
}
EntryProbeResult DispatchProbe(EntryCoordinator& entry, FrontierExplorer& explorer,
                               RoomVisitLedger& ledger, EntryProbeActions& actions,
                               EntryProbeObservation observation, EntryNotice& notice) {
  const auto& target = notice.target;
  // The attempt counted for the inside check is also this probe attempt.
  entry.NoteEvidence({explorer.KnownCells(), observation.map_revision});
  // Reopen unknown/free trail evidence only; never inherit cooled hallway
  // chords. Remember the room even when the subsequent plan is rejected.
  actions.PrepareEntryPlan(4.);
  ledger.Remember(*entry.pending(), target);
  notice.attempts = entry.attempts();
  if (std::hypot(target.x - observation.robot_x, target.y - observation.robot_y) <=
      observation.goal_tolerance_m) {
    notice.kind = EntryNoticeKind::kTooNear;
    actions.ReportEntry(notice);
    return EntryProbeResult::kRejected;
  }
  if (!actions.NavigateEntry(target)) {
    notice.kind = EntryNoticeKind::kPlanRejected;
    actions.ReportEntry(notice);
    return EntryProbeResult::kRejected;
  }
  // Read pending after navigation, as in the original adapter.
  notice.mouth = *entry.pending();
  notice.kind = EntryNoticeKind::kProbing;
  actions.ReportEntry(notice);
  return EntryProbeResult::kDriving;
}
}  // namespace
EntryProbeResult ExecuteEntryProbe(EntryCoordinator& entry, FrontierExplorer& explorer,
                                   RoomVisitLedger& ledger, EntryProbeActions& actions,
                                   EntryProbeObservation observation) {
  if (!entry.pending()) {
    return EntryProbeResult::kIdle;
  }
  if (actions.EntryMouthCooling(*entry.pending())) {
    return EntryProbeResult::kCooling;
  }
  // Keep one mouth committed instead of hopping between adjacent frontiers.
  if (!entry.locked() || !EntryCoordinator::SameMouth(*entry.locked(), *entry.pending())) {
    const auto mouth = *entry.pending();
    const bool easy = explorer.IsCorridorMouth(mouth.frontier_x, mouth.frontier_y) &&
                      explorer.MouthGapWidth(mouth.frontier_x, mouth.frontier_y) >= .50 &&
                      explorer.MouthGapWidth(mouth.frontier_x, mouth.frontier_y) <= 1.40;
    entry.Lock(mouth, easy, {explorer.KnownCells(), observation.map_revision});
  } else {
    entry.SetPending(*entry.locked());
  }
  if (Approach(*entry.pending(), actions, observation)) {
    return EntryProbeResult::kDriving;
  }
  const auto selection = SelectEntryTarget(explorer, *entry.pending(), observation.robot_x,
                                           observation.robot_y, entry.attempts());
  EntryNotice notice{EntryNoticeKind::kUnavailable, *entry.pending(), {}};
  notice.minimum_depth_m = selection.minimum_depth_m;
  notice.gap_m = selection.gap_m;
  notice.free_depth_m = selection.free_depth_m;
  notice.easy = selection.easy;
  if (!selection.goal) {
    entry.CountAttempt();
    notice.attempts = entry.attempts();
    actions.ReportEntry(notice);
    return EntryProbeResult::kUnavailable;
  }
  notice.target = *selection.goal;
  entry.CountAttempt();
  if (CloseIfEntered(entry, explorer, ledger, actions, observation, notice)) {
    return EntryProbeResult::kEntered;
  }
  return DispatchProbe(entry, explorer, ledger, actions, observation, notice);
}
}  // namespace navigation2d::exploration
