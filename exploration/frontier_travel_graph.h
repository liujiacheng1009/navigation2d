#pragma once
#include <optional>
#include <vector>

#include "navigation2d/exploration/frontier_types.h"
namespace navigation2d::frontier_internal {
struct FrontierTravelGraph {
  // Execution-authorized viewpoint mask and discovery distances are distinct.
  // A nonnegative travel distance alone never authorizes a navigation goal.
  std::vector<unsigned char> safe;
  std::vector<int> travel_cells;
};
struct FrontierTravelOrigin {
  int col, row, search_radius_cells;
};
// Maps have valid matching dimensions/storage and positive resolution. Reads
// synchronously; returned buffers own their storage. Nullopt: no safe start
// within the original one-metre square. Row-major ties and 4-neighbour order
// match the execution graph; discovery only fills otherwise unreachable cells.
[[nodiscard]] std::optional<FrontierTravelGraph> BuildFrontierTravelGraph(
    const ExplorationGrid& planning, const ExplorationGrid& observation,
    double footprint_clearance_m, double robot_x, double robot_y);
// Internal bootstrap policy: a small observed map may use the raw free graph
// to discover boundaries; the independent safe mask is never modified.
// The distance buffer has planning.cells.size() entries; origin uses grid cells.
void ApplyBootstrapFrontierTravel(const ExplorationGrid& planning,
                                  const ExplorationGrid& observation, FrontierTravelOrigin,
                                  std::vector<int>& distances);
}  // namespace navigation2d::frontier_internal
