#pragma once
#include <cstdint>

#include "navigation2d/application/exploration/entry_actions.h"
#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/room_visit_ledger.h"

namespace navigation2d::exploration {
enum class EntryProbeResult { kIdle, kCooling, kDriving, kEntered, kUnavailable, kRejected };
struct EntryProbeObservation {
  double robot_x, robot_y;  // World-frame metres.
  std::uint64_t map_revision;
  double goal_tolerance_m;
};
// No retained services or duplicate mission state. EntryCoordinator and ledger
// remain the sole state owners; map queries stay in FrontierExplorer.
EntryProbeResult ExecuteEntryProbe(EntryCoordinator&, FrontierExplorer&, RoomVisitLedger&,
                                   EntryActions&, EntryProbeObservation);
}  // namespace navigation2d::exploration
