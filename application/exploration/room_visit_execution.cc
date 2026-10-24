#include "navigation2d/application/exploration/room_visit_execution.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
bool TryUnenteredRoomVisits(RoomVisitLedger& visits, FrontierExplorer& explorer,
                            const std::vector<std::pair<double, double>>& trail, double rx,
                            double ry, RoomVisitActions& actions) {
  if (visits.empty()) {
    return false;
  }
  const auto room_seeds = visits.Seeds();
  auto goals = explorer.UnvisitedSpaceGoals(
      trail, navigation2d::kOpenRetreat(),
      navigation2d::kRoomPocketDepthM() * navigation2d::kRoomPocketDepthM(), room_seeds,
      navigation2d::kOpenRetreat());
  std::sort(goals.begin(), goals.end(), [&](const ExplorationGoal& a, const ExplorationGoal& b) {
    return std::hypot(a.x - rx, a.y - ry) < std::hypot(b.x - rx, b.y - ry);
  });
  for (auto& goal : goals) {
    const int attempt = visits.TryVisit(goal);
    if (attempt == 0) {
      continue;
    }
    goal.yaw = std::atan2(goal.y - ry, goal.x - rx);
    actions.PrepareRoomVisit();
    const bool accepted = actions.NavigateRoomVisit(goal);
    actions.ReportRoomVisit(goal, attempt, accepted);
    if (accepted) {
      return true;
    }
  }
  return false;
}

}  // namespace navigation2d::exploration
