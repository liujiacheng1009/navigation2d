#include "navigation2d/application/exploration/continuous_tour_handoff.h"

#include <cmath>
#include <utility>

#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::exploration {
namespace {
double NormalizeAngle(double value) { return std::atan2(std::sin(value), std::cos(value)); }
}  // namespace
bool TryContinuousTourHandoff(TourExecution& tour, FrontierExplorer& explorer,
                              NavigationSession& navigation, const ExplorationGoal& active_goal,
                              TourPose robot, ContinuousHandoffActions& actions) {
  const double distance = std::hypot(active_goal.x - robot.x, active_goal.y - robot.y);
  // A frontier viewpoint is a 360-degree observation pose, not a docking
  // point. Once inside this small observation envelope, the lidar has the
  // intended view and the next already-ranked task can be adopted without
  // braking to zero at every frontier boundary.
  if (distance > .24) {
    return false;
  }
  const ExplorationGoal completed = active_goal;
  std::size_t next_index = 1;
  // Match BuildTour's ~5 m pocket: cross-hall handoff must stop at a decision
  // boundary so newly opened junction mouths are not skipped.
  constexpr double kHandoffPocketM = 5.0;
  while (next_index < tour.goals().size()) {
    const ExplorationGoal next = tour.goals()[next_index];
    if (!explorer.GoalRegionStillFrontier(next)) {
      ++next_index;
      continue;
    }
    if (!explorer.InSameReachableRegion(robot.x, robot.y, next.x, next.y, kHandoffPocketM) &&
        !explorer.InSameReachableRegion(active_goal.x, active_goal.y, next.x, next.y,
                                        kHandoffPocketM)) {
      return false;
    }
    const double next_bearing = std::atan2(next.y - robot.y, next.x - robot.x);
    const double direction_change = std::abs(NormalizeAngle(next_bearing - robot.yaw));
    // Continuous handoff is valid only when the next task continues the
    // current motion direction. Switching a moving differential drive to
    // a goal behind it creates an artificial emergency stop and lets
    // inertia carry the base outside the newly planned corridor.
    if (direction_change > .60) {
      return false;
    }
    auto previous = std::move(navigation);
    if (actions.NavigateHandoff(next)) {
      explorer.RecordAttempt(completed, true);
      // Drop the completed viewpoint and any resolved entries before the
      // newly active goal; leave that goal at the front for normal result
      // bookkeeping after it is reached.
      tour.HandoffThrough(next_index);
      actions.HandoffAccepted();
      return true;
    }
    navigation = std::move(previous);
    explorer.RecordAttempt(next, false);
    ++next_index;
  }
  return false;
}

}  // namespace navigation2d::exploration
