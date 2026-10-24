#include "navigation2d/application/exploration/tour_execution.h"

#include <cmath>
#include <utility>
namespace navigation2d::exploration {
using Goal = ExplorationGoal;
void TourExecution::Build(FrontierExplorer& explorer, TourPose robot) {
  goals_.clear();
  const auto goals = explorer.BuildTour(robot.x, robot.y, robot.yaw);
  for (const auto& goal : goals) {
    goals_.push_back(goal);
  }
  ++revision_;
}
void TourExecution::QueueActiveRoom(FrontierExplorer& explorer, TourPose robot,
                                    TourConstraints& constraints) {
  const double rx = robot.x, ry = robot.y;
  explorer.CommitRegion(rx, ry);
  goals_.clear();
  auto leftovers = explorer.SelectLeftoverApproaches(rx, ry, 6);
  leftovers = explorer.OrderByRegion(std::move(leftovers), rx, ry, robot.yaw);
  for (auto& goal : leftovers) {
    if (!explorer.GoalInActiveRegion(goal)) {
      continue;
    }
    if (constraints.RecentlyDemoted(goal)) {
      continue;
    }
    goals_.push_back(std::move(goal));
  }
  if (goals_.empty()) {
    auto mouths = explorer.SelectGoals(rx, ry);
    mouths = explorer.OrderByRegion(std::move(mouths), rx, ry, robot.yaw);
    for (auto& goal : mouths) {
      if (!explorer.GoalInActiveRegion(goal)) {
        continue;
      }
      if (constraints.RecentlyDemoted(goal)) {
        continue;
      }
      goals_.push_back(std::move(goal));
    }
  }
}
void TourExecution::Commit(TourOrder&& order) {
  goals_.clear();
  for (auto& goal : order.priority_local) {
    goals_.push_back(std::move(goal));
  }
  for (auto& goal : order.far) {
    goals_.push_back(std::move(goal));
  }
  if (order.deferred_local) {
    for (auto& goal : *order.deferred_local) {
      goals_.push_back(std::move(goal));
    }
  }
}
void TourExecution::AfterMouthFailure(FrontierExplorer& explorer, const Goal& mouth,
                                      std::optional<TourPose> robot, TourConstraints& constraints) {
  Commit(MouthTourOrder(goals_, explorer, mouth, robot, constraints));
}
void TourExecution::AfterWallTip(FrontierExplorer& explorer, const Goal& tip,
                                 std::optional<TourPose> robot, TourConstraints& constraints) {
  Commit(WallTipTourOrder(goals_, explorer, tip, robot, constraints));
}
bool TourExecution::PrioritizeDistant(FrontierExplorer& explorer, TourPose robot,
                                      TourConstraints& constraints) {
  const double rx = robot.x, ry = robot.y;
  std::deque<Goal> distant;
  for (const auto& goal : goals_) {
    if (constraints.RecentlyDemoted(goal)) {
      continue;
    }
    if (std::hypot(goal.x - rx, goal.y - ry) < 1.8) {
      continue;
    }
    distant.push_back(goal);
  }
  auto leftovers = explorer.SelectLeftoverApproaches(rx, ry, 8);
  leftovers = explorer.OrderByRegion(std::move(leftovers), rx, ry, robot.yaw);
  for (auto& goal : leftovers) {
    if (std::hypot(goal.x - rx, goal.y - ry) < 1.5) {
      continue;
    }
    if (!explorer.IsCorridorMouth(goal.frontier_x, goal.frontier_y) &&
        explorer.MouthGapWidth(goal.frontier_x, goal.frontier_y) < .50) {
      continue;
    }
    distant.push_front(std::move(goal));
  }
  goals_.clear();
  for (auto& goal : distant) {
    goals_.push_back(std::move(goal));
  }
  return !goals_.empty();
}
void TourExecution::SkipResolved(FrontierExplorer& explorer) {
  while (!goals_.empty() && !explorer.GoalRegionStillFrontier(goals_.front())) {
    goals_.pop_front();
  }
}
void TourExecution::HandoffThrough(std::size_t count) {
  for (std::size_t index = 0; index < count; ++index) {
    goals_.pop_front();
  }
}
}  // namespace navigation2d::exploration
