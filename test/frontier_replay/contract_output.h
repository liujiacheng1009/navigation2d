#pragma once
#include <iomanip>
#include <iostream>
#include <optional>
#include <vector>

#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::test {
inline void GoalFields(const ExplorationGoal& g) {
  std::cout << '[' << g.x << ',' << g.y << ',' << g.yaw << ',' << g.frontier_x << ','
            << g.frontier_y << ',' << g.frontier_cells << ',' << g.information_gain << ','
            << g.score << ',' << g.leftover_approach << ',' << g.region_id << ']';
}
inline void Goals(const std::vector<ExplorationGoal>& goals) {
  std::cout << '[';
  bool first = true;
  for (const auto& goal : goals) {
    if (!first) {
      std::cout << ',';
    }
    GoalFields(goal);
    first = false;
  }
  std::cout << ']';
}
inline void DebugGoals(const std::vector<FrontierDebugSnapshot::Goal>& goals) {
  std::cout << '[';
  bool first = true;
  for (const auto& goal : goals) {
    if (!first) {
      std::cout << ',';
    }
    std::cout << '[' << goal.x << ',' << goal.y << ',' << goal.frontier_x << ',' << goal.frontier_y
              << ',' << goal.cluster_id << ',' << goal.score << ',' << goal.information_gain << ','
              << goal.frontier_cells << ']';
    first = false;
  }
  std::cout << ']';
}
inline void OptionalGoal(const std::optional<ExplorationGoal>& goal) {
  if (goal) {
    GoalFields(*goal);
  } else {
    std::cout << "null";
  }
}
}  // namespace navigation2d::test
