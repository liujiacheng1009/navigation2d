#pragma once
#include <optional>

#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d::frontier_internal {
// A world-frame goal already safe on original free evidence is unchanged.
// Otherwise move only x/y to the nearest permitted standoff cell, retaining
// row-major strict ties and all other goal identity/metadata.
[[nodiscard]] std::optional<ExplorationGoal> SettleViewpoint(const ExplorationGrid& planning,
                                                             const ExplorationGrid& observation,
                                                             double footprint_clearance,
                                                             const ExplorationGoal&);
}  // namespace navigation2d::frontier_internal
