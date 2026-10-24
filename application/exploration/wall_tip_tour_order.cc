#include <algorithm>
#include <cmath>
#include <utility>

#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/tour_order.h"
namespace navigation2d::exploration {
using Goal = ExplorationGoal;
TourOrder WallTipTourOrder(const std::deque<Goal>& current, FrontierExplorer& explorer,
                           const Goal& tip, std::optional<TourPose> robot,
                           TourConstraints& constraints) {
  constexpr double kLocalHopM = 4.0;
  const double rx = robot ? robot->x : tip.x;
  const double ry = robot ? robot->y : tip.y;
  const auto is_local = [&](const Goal& goal) {
    return std::hypot(goal.x - rx, goal.y - ry) <= kLocalHopM ||
           std::hypot(goal.frontier_x - tip.frontier_x, goal.frontier_y - tip.frontier_y) <=
               kLocalHopM;
  };
  TourOrder result;
  auto& local = result.priority_local;
  auto& far = result.far;
  for (const auto& goal : current) {
    if (EntryCoordinator::SameMouth(goal, tip) || constraints.RecentlyDemoted(goal)) {
      continue;
    }
    if (is_local(goal)) {
      local.push_back(goal);
    } else {
      far.push_back(goal);
    }
  }
  if (robot) {
    auto mouths = explorer.SelectGoals(rx, ry);
    mouths = explorer.OrderByRegion(std::move(mouths), rx, ry, robot->yaw);
    for (const auto& candidate : mouths) {
      if (EntryCoordinator::SameMouth(candidate, tip) || constraints.RecentlyDemoted(candidate)) {
        continue;
      }
      if (!is_local(candidate)) {
        continue;
      }
      const bool duplicate = std::any_of(local.begin(), local.end(), [&](const Goal& prior) {
        return EntryCoordinator::SameMouth(prior, candidate);
      });
      if (!duplicate) {
        local.push_back(candidate);
      }
    }
    if (local.empty()) {
      for (const auto& candidate : mouths) {
        if (EntryCoordinator::SameMouth(candidate, tip) || constraints.RecentlyDemoted(candidate)) {
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
