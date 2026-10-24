#include <algorithm>
#include <cmath>

#include "navigation2d/exploration/mouth_goal_stages.h"
namespace navigation2d::frontier_internal {
std::vector<ExplorationGoal> MergeMouthGoals(std::vector<ExplorationGoal> goals) {
  // Spatial hypercluster: micro-cluster frontiers within ~1.75 m are one mouth
  // (doorway / alcove). Only the strongest viewpoint survives; then prefer an
  // entry pose on the unknown side when the planning graph permits it.
  constexpr double kMouthMergeM = 1.5;
  const auto better = [](const ExplorationGoal& a, const ExplorationGoal& b) {
    if (a.information_gain > b.information_gain + 1e-9) {
      return true;
    }
    if (b.information_gain > a.information_gain + 1e-9) {
      return false;
    }
    if (a.frontier_cells != b.frontier_cells) {
      return a.frontier_cells > b.frontier_cells;
    }
    return a.score > b.score;
  };
  std::vector<ExplorationGoal> mouths;
  mouths.reserve(goals.size());
  for (const auto& goal : goals) {
    bool absorbed = false;
    for (auto& keep : mouths) {
      if (std::hypot(keep.frontier_x - goal.frontier_x, keep.frontier_y - goal.frontier_y) >
              kMouthMergeM &&
          std::hypot(keep.x - goal.x, keep.y - goal.y) > kMouthMergeM) {
        continue;
      }
      if (better(goal, keep)) {
        const int cells = std::max(keep.frontier_cells, goal.frontier_cells);
        const double gain = std::max(keep.information_gain, goal.information_gain);
        keep = goal;
        keep.frontier_cells = cells;
        keep.information_gain = gain;
      } else {
        keep.frontier_cells = std::max(keep.frontier_cells, goal.frontier_cells);
        keep.information_gain = std::max(keep.information_gain, goal.information_gain);
      }
      absorbed = true;
      break;
    }
    if (!absorbed) {
      mouths.push_back(goal);
    }
  }
  return mouths;
}

}  // namespace navigation2d::frontier_internal
