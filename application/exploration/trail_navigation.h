#pragma once
#include <optional>
#include <span>
#include <utility>

#include "navigation2d/application/exploration/tour_order.h"
namespace navigation2d::exploration {
using DrivenTrail = std::span<const std::pair<double, double>>;
// World-frame geometry in metres. Dense samples do not count as displacement.
[[nodiscard]] std::optional<ExplorationGoal> SelectTrailRetreat(DrivenTrail, TourPose, bool entry,
                                                                bool pinched);
[[nodiscard]] bool GoalBehindDrivenTrail(DrivenTrail, TourPose, const ExplorationGoal&);
}  // namespace navigation2d::exploration
