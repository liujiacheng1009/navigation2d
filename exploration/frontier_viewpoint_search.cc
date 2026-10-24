#include "navigation2d/exploration/frontier_viewpoint_search.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/exploration/doorway_geometry.h"
#include "navigation2d/exploration/frontier_viewpoint_score.h"
#include "navigation2d/exploration/frontier_visibility.h"
namespace navigation2d::frontier_internal {
bool NarrowViewpoint(const FrontierViewpointProblem& problem, int col, int row) {
  const auto& map = problem.map;
  const auto geometry = problem.Geometry();
  // Match SafeViewpoint / CorridorMouthAt: kitchen vestibules are often
  // narrow on BOTH axes. XOR here dropped every both-narrow mouth so
  // alcove/room frontiers stayed raw_cells>0 with candidates=0.
  const int probe = static_cast<int>(std::ceil(1.4 / map.resolution));
  int horizontal = 0, vertical = 0;
  for (int x = col; x >= 0 && x > col - probe && geometry.Free(x, row); --x) {
    ++horizontal;
  }
  for (int x = col + 1; x < map.width && x <= col + probe && geometry.Free(x, row); ++x) {
    ++horizontal;
  }
  for (int y = row; y >= 0 && y > row - probe && geometry.Free(col, y); --y) {
    ++vertical;
  }
  for (int y = row + 1; y < map.height && y <= row + probe && geometry.Free(col, y); ++y) {
    ++vertical;
  }
  const int narrow_min = static_cast<int>(std::ceil(kNarrowMinWidth() / map.resolution));
  const int narrow_max = static_cast<int>(std::floor(kNarrowMaxWidth() / map.resolution));
  const bool narrow_h = horizontal >= narrow_min && horizontal <= narrow_max;
  const bool narrow_v = vertical >= narrow_min && vertical <= narrow_max;
  const int smaller = std::min(horizontal, vertical);
  return narrow_h || narrow_v || (smaller >= narrow_min && smaller <= narrow_max);
}
std::optional<ExplorationGoal> MakeComponentCandidate(const FrontierViewpointProblem& problem,
                                                      int col, int row, double target_x,
                                                      double target_y, double standoff,
                                                      double visibility_weight) {
  const auto& map = problem.map;
  const double clearance = OccupiedClearanceAt(map, col, row);
  if (clearance < kPlanningRadius()) {
    return std::nullopt;
  }
  const double travel = problem.travel_cells[row * map.width + col] * map.resolution;
  const double information_gain =
      std::max(VisibleUnknownGain(problem.observation, map.resolution, col, row),
               static_cast<double>(problem.cluster.size()) * map.resolution * map.resolution);
  const auto [x, y] = problem.Geometry().CellCenter(col, row);
  return ExplorationGoal{
      x,
      y,
      std::atan2(target_y - y, target_x - x),
      problem.anchor.frontier_x,
      problem.anchor.frontier_y,
      static_cast<int>(problem.cluster.size()),
      information_gain,
      ViewpointScore(information_gain, travel, standoff, visibility_weight, clearance)};
}
std::optional<ExplorationGoal> SelectComponentViewpoint(const FrontierViewpointProblem& problem) {
  if (auto goal = SearchVisibleViewpoint(problem, false)) {
    return goal;
  }
  if (auto goal = SearchVisibleViewpoint(problem, true)) {
    return goal;
  }
  if (auto goal = SearchNarrowViewpoint(problem)) {
    return goal;
  }
  return SearchOpenApproach(problem);
}
}  // namespace navigation2d::frontier_internal
