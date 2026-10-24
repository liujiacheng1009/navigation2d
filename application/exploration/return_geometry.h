#pragma once
#include <span>
#include <utility>

#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d::exploration {
enum class ReturnStepKind { kDirect, kBreadcrumb, kRepeated };
struct ReturnStep {
  ReturnStepKind kind;
  ExplorationGoal goal;
};
struct ReturnPosition {
  double x, y;
};
// Read-only map-frame geometry, metres/radians. Unknown is not occupied.
ExplorationGoal SnapReturnHome(const ExplorationGrid* map, ExplorationGoal home);
// Retains nearest-sample tie breaks and reverse traversal order. No budgets or
// mission transitions are changed; breadcrumbs is a read-only state snapshot.
ReturnStep SelectReturnStep(std::span<const std::pair<double, double>> trail,
                            const ExplorationGoal& home, ReturnPosition robot,
                            const ExplorationGoal& active_goal, bool breadcrumbs);
}  // namespace navigation2d::exploration
