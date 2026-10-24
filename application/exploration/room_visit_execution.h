#pragma once
#include <utility>
#include <vector>

#include "navigation2d/application/exploration/room_visit_ledger.h"
namespace navigation2d {
class FrontierExplorer;
}
namespace navigation2d::exploration {
class RoomVisitActions {
 public:
  virtual ~RoomVisitActions() = default;
  virtual void PrepareRoomVisit() = 0;
  virtual bool NavigateRoomVisit(const ExplorationGoal&) = 0;
  virtual void ReportRoomVisit(const ExplorationGoal&, int attempt, bool accepted) = 0;
};
// Caller gates returning/missing-pose states. Map/world-frame metres, synchronous
// read-only trail. Attempt consumption precedes preparation/planning/reporting.
bool TryUnenteredRoomVisits(RoomVisitLedger&, FrontierExplorer&,
                            const std::vector<std::pair<double, double>>& trail, double robot_x,
                            double robot_y, RoomVisitActions&);
}  // namespace navigation2d::exploration
