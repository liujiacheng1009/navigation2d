#include "navigation2d/exploration/frontier_explorer.h"
#include <algorithm>
#include <cmath>
#include <utility>

#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/tour_order.h"
namespace navigation2d::exploration {
using Goal = ExplorationGoal;
TourOrder MouthTourOrder(const std::deque<Goal>& current, FrontierExplorer& explorer,
                         const Goal& mouth, std::optional<TourPose> robot,
                         TourConstraints& constraints) {
  constexpr double kLocalHopM = 4.0;
  const double rx = robot ? robot->x : mouth.x;
  const double ry = robot ? robot->y : mouth.y;
  const auto is_local = [&](const Goal& goal) {
    return std::hypot(goal.x - rx, goal.y - ry) <= kLocalHopM ||
           std::hypot(goal.frontier_x - mouth.frontier_x, goal.frontier_y - mouth.frontier_y) <=
               kLocalHopM ||
           explorer.GoalInActiveRegion(goal);
  };
  TourOrder result;
  result.deferred_local.emplace();
  auto& ahead_local = result.priority_local;
  auto& behind_local = *result.deferred_local;
  auto& far = result.far;
  const auto place = [&](Goal goal) {
    if (EntryCoordinator::SameMouth(goal, mouth) || constraints.RecentlyDemoted(goal)) {
      return;
    }
    if (!is_local(goal)) {
      far.push_back(std::move(goal));
      return;
    }
    if (constraints.GoalIsBehindTrail(goal)) {
      behind_local.push_back(std::move(goal));
    } else {
      ahead_local.push_back(std::move(goal));
    }
  };
  for (const auto& goal : current) {
    place(goal);
  }
  // Pull in other executable mouths near the abandoned doorway.
  if (robot) {
    auto mouths = explorer.SelectGoals(rx, ry);
    mouths = explorer.OrderByRegion(std::move(mouths), rx, ry, robot->yaw);
    for (const auto& candidate : mouths) {
      if (EntryCoordinator::SameMouth(candidate, mouth) || constraints.RecentlyDemoted(candidate)) {
        continue;
      }
      const bool duplicate =
          std::any_of(
              ahead_local.begin(), ahead_local.end(),
              [&](const Goal& prior) { return EntryCoordinator::SameMouth(prior, candidate); }) ||
          std::any_of(
              behind_local.begin(), behind_local.end(),
              [&](const Goal& prior) { return EntryCoordinator::SameMouth(prior, candidate); }) ||
          std::any_of(far.begin(), far.end(), [&](const Goal& prior) {
            return EntryCoordinator::SameMouth(prior, candidate);
          });
      if (duplicate) {
        continue;
      }
      place(candidate);
    }
    if (ahead_local.empty() && behind_local.empty() && far.empty()) {
      for (const auto& candidate : mouths) {
        if (EntryCoordinator::SameMouth(candidate, mouth) ||
            constraints.RecentlyDemoted(candidate)) {
          continue;
        }
        far.push_back(candidate);
        break;
      }
    }
  }
  return result;
}
}  // namespace navigation2d::exploration
