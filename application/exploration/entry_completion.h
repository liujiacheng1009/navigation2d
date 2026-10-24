#pragma once
#include "navigation2d/application/exploration/entry_actions.h"
#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/room_visit_ledger.h"
namespace navigation2d::exploration {
// Requires a pending mouth. Preserve each arrival source's cooling order;
// acknowledging an entrance never marks its whole region complete.
void CompleteEntryArrival(EntryCoordinator&, FrontierExplorer&, RoomVisitLedger&, EntryActions&,
                          EntryArrival, double robot_x, double robot_y);
}  // namespace navigation2d::exploration
