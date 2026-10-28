#pragma once
#include <map>
#include <span>

#include "navigation2d/exploration/region_types.h"
namespace navigation2d {
struct ExplorationGrid;
namespace exploration_internal {
// One complete graph/evidence observation over already assigned identities.
// Labels match the grid cell count and every nonzero label exists in regions.
// The scheduler owns records; no input or mutable reference is retained.
// Resets transient graph facts, then updates evidence-based completion/reopen.
void ObserveRegionGraph(const ExplorationGrid&, std::span<const RegionId> labels,
                        std::map<RegionId, RegionRecord>& regions);
}  // namespace exploration_internal
}  // namespace navigation2d
