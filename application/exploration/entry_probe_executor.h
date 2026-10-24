#pragma once
#include <cstdint>

#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/entry_target.h"
#include "navigation2d/application/exploration/room_visit_ledger.h"

namespace navigation2d::exploration {
enum class EntryProbeResult { kIdle, kCooling, kDriving, kEntered, kUnavailable, kRejected };
enum class EntryNoticeKind {
  kApproaching,
  kUnavailable,
  kEntered,
  kTooNear,
  kPlanRejected,
  kProbing
};
struct EntryNotice {
  EntryNoticeKind kind;
  ExplorationGoal mouth, target;
  double robot_x = 0., robot_y = 0., range_m = 0.;
  double minimum_depth_m = 0., gap_m = 0., free_depth_m = 0.;
  int attempts = 0;
  bool easy = false;
};
struct EntryProbeObservation {
  double robot_x, robot_y;  // World-frame metres.
  std::uint64_t map_revision;
  double goal_tolerance_m;
};
// Synchronous capabilities. Implementations must not defer calls or retain
// references. The executor owns ordering; adapters own navigation and outputs.
class EntryProbeActions {
 public:
  virtual ~EntryProbeActions() = default;
  virtual bool EntryMouthCooling(const ExplorationGoal&) = 0;
  virtual void PrepareEntryPlan(double trail_budget_m) = 0;
  virtual bool NavigateEntry(const ExplorationGoal&) = 0;
  virtual void CoolEnteredMouth(const ExplorationGoal&, double seconds) = 0;
  virtual void QueueActiveRegionFinish() = 0;
  virtual void SelectAfterEntry() = 0;
  virtual void ReportEntry(const EntryNotice&) = 0;
};
// No retained services or duplicate mission state. EntryCoordinator and ledger
// remain the sole state owners; map queries stay in FrontierExplorer.
EntryProbeResult ExecuteEntryProbe(EntryCoordinator&, FrontierExplorer&, RoomVisitLedger&,
                                   EntryProbeActions&, EntryProbeObservation);
}  // namespace navigation2d::exploration
