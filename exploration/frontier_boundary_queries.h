#pragma once

#include "navigation2d/exploration/frontier_types.h"

namespace navigation2d::frontier_internal {
// Read-only, synchronous evidence queries in world coordinates. Frontier and
// unentered-room identity use mapper observation only; free-depth uses planning
// geometry. Inputs outlive each call; valid dimensions must match cell storage.
bool GoalStillFrontierIn(const ExplorationGrid& observation, const ExplorationGoal& goal);
bool GoalRegionStillFrontierIn(const ExplorationGrid& observation, const ExplorationGoal& goal);
bool UnenteredRoomBeyondIn(const ExplorationGrid& observation, const ExplorationGoal& goal);
double MouthFreeDepthIn(const ExplorationGrid& planning, const ExplorationGrid& observation,
                        double footprint_clearance, const ExplorationGoal& frontier_goal,
                        double robot_x, double robot_y);
}  // namespace navigation2d::frontier_internal
