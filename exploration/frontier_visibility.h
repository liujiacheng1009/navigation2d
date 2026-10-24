#pragma once
#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d::frontier_internal {
// 144 rays, 3 m in planning-cell steps; only original observation authorizes
// gain, measured with observation-cell area. Occupied endpoints stop rays.
[[nodiscard]] double VisibleUnknownGain(const ExplorationGrid& observation,
                                        double planning_resolution, int col, int row);
}  // namespace navigation2d::frontier_internal
