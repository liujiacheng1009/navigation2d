#include "navigation2d/application/exploration/trail_navigation.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
std::optional<ExplorationGoal> SelectTrailRetreat(DrivenTrail trail, TourPose robot, bool entry,
                                                  bool pinched) {
  if (trail.size() < 4) {
    return std::nullopt;
  }
  const double want = entry ? kEntryRetreat() : (pinched ? kPocketRetreat() : kOpenRetreat());
  std::size_t index = trail.size() - 1, best = index;
  double best_distance = 0.;
  while (index > 0) {
    --index;
    const double distance = std::hypot(trail[index].first - robot.x, trail[index].second - robot.y);
    if (distance > best_distance) {
      best_distance = distance;
      best = index;
    }
    if (distance >= want) {
      break;
    }
  }
  if (best_distance < .5 * want) {
    return std::nullopt;
  }
  ExplorationGoal goal;
  goal.x = trail[best].first;
  goal.y = trail[best].second;
  goal.yaw = std::atan2(goal.y - robot.y, goal.x - robot.x);
  return goal;
}
bool GoalBehindDrivenTrail(DrivenTrail trail, TourPose robot, const ExplorationGoal& goal) {
  if (trail.size() < 8) {
    return false;
  }
  const double to_goal = std::hypot(goal.x - robot.x, goal.y - robot.y);
  if (to_goal < .35) {
    return false;
  }
  double best_behind = 1e9, walked = 0.;
  for (std::size_t i = trail.size(); i-- > 0;) {
    if (i + 1 < trail.size()) {
      walked +=
          std::hypot(trail[i].first - trail[i + 1].first, trail[i].second - trail[i + 1].second);
    }
    if (walked < 1.20) {
      continue;
    }
    if (walked > 8.0) {
      break;
    }
    const double distance = std::hypot(goal.x - trail[i].first, goal.y - trail[i].second);
    best_behind = std::min(best_behind, distance);
  }
  return best_behind + .60 < to_goal;
}
}  // namespace navigation2d::exploration
