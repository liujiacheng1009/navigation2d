#include "navigation2d/application/exploration/room_visit_ledger.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
void RoomVisitLedger::Remember(const ExplorationGoal& frontier, const ExplorationGoal& probe) {
  for (const auto& room : rooms_) {
    if (std::hypot(room.frontier_x - frontier.frontier_x, room.frontier_y - frontier.frontier_y) <
        navigation2d::kSwingClearance()) {
      return;
    }
  }
  rooms_.push_back({frontier.frontier_x, frontier.frontier_y, probe.x, probe.y});
}

void RoomVisitLedger::Forget(const ExplorationGoal& frontier) {
  rooms_.erase(std::remove_if(rooms_.begin(), rooms_.end(),
                              [&](const UnenteredRoom& room) {
                                return std::hypot(room.frontier_x - frontier.frontier_x,
                                                  room.frontier_y - frontier.frontier_y) <
                                       navigation2d::kSwingClearance();
                              }),
               rooms_.end());
}

std::vector<std::pair<double, double>> RoomVisitLedger::Seeds() const {
  std::vector<std::pair<double, double>> seeds;
  for (const auto& room : rooms_) {
    seeds.emplace_back(room.probe_x, room.probe_y);
  }
  return seeds;
}
int RoomVisitLedger::TryVisit(const ExplorationGoal& goal) {
  auto attempt = std::find_if(attempts_.begin(), attempts_.end(), [&](const VisitAttempt& a) {
    return std::hypot(a.x - goal.x, a.y - goal.y) < kSwingClearance();
  });
  if (attempt == attempts_.end()) {
    attempts_.push_back({goal.x, goal.y, 0});
    attempt = std::prev(attempts_.end());
  }
  if (attempt->count >= kVisitAttempts) {
    return 0;
  }
  return ++attempt->count;
}
}  // namespace navigation2d::exploration
