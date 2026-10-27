#pragma once
#include "navigation2d/application/exploration/exploration_mission.h"
namespace navigation2d::exploration {
// Source compatibility for the lifecycle-only API; state has one owner.
using MissionLifecycle = ExplorationMission;
}  // namespace navigation2d::exploration
