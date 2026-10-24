#pragma once
#include <utility>
#include <vector>

#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
// One writer owns entrance evidence and per-spot retry counts for a mission.
class RoomVisitLedger {
 public:
  static constexpr int kVisitAttempts = 2;
  void Remember(const ExplorationGoal& frontier, const ExplorationGoal& probe);
  void Forget(const ExplorationGoal& frontier);
  bool empty() const { return rooms_.empty(); }
  std::vector<std::pair<double, double>> Seeds() const;
  // Consumes an attempt before planning, as in the original node. Zero means
  // exhausted. Nearby spots share the original strict swing-radius budget.
  int TryVisit(const ExplorationGoal& goal);

 private:
  struct UnenteredRoom {
    double frontier_x, frontier_y, probe_x, probe_y;
  };
  struct VisitAttempt {
    double x, y;
    int count;
  };
  std::vector<UnenteredRoom> rooms_;
  std::vector<VisitAttempt> attempts_;
};
}  // namespace navigation2d::exploration
