#include <algorithm>
#include <cmath>

#include "navigation2d/exploration/doorway_geometry.h"
#include "navigation2d/exploration/entry_probe_search.h"
#include "navigation2d/exploration/mouth_goal_stages.h"
namespace navigation2d::frontier_internal {
namespace {
double DepthPastMouth(const ExplorationGoal& goal, double x, double y, double robot_x,
                      double robot_y) {
  const double to_fx = goal.frontier_x - robot_x;
  const double to_fy = goal.frontier_y - robot_y;
  const double length = std::hypot(to_fx, to_fy);
  if (length < .05) {
    return 0.;
  }
  return ((x - goal.frontier_x) * to_fx + (y - goal.frontier_y) * to_fy) / length;
}

void CenterMouthGoal(ExplorationGoal& goal, const ExplorationGrid& map,
                     const ExplorationGrid& observation, double footprint_clearance, double robot_x,
                     double robot_y) {
  const PlanningGridView geometry(map, observation, footprint_clearance);
  const ObservationGridView observed(observation);
  const double to_fx = goal.frontier_x - robot_x;
  const double to_fy = goal.frontier_y - robot_y;
  const double length = std::hypot(to_fx, to_fy);
  if (length < .05) {
    return;
  }
  const double dir_x = to_fx / length;
  const double dir_y = to_fy / length;
  const int search = static_cast<int>(std::ceil(.70 / map.resolution));
  const int fcol = static_cast<int>(std::floor((goal.frontier_x - map.origin_x) / map.resolution));
  const int frow = static_cast<int>(std::floor((goal.frontier_y - map.origin_y) / map.resolution));
  // Mid-gap: maximize clearance (not depth). Old depth-max parked the
  // standoff on the deeper jamb side so RPP froze at an easy doorway.
  double best_score = -1.;
  int best_col = -1, best_row = -1;
  for (int row = std::max(0, frow - search); row <= std::min(map.height - 1, frow + search);
       ++row) {
    for (int col = std::max(0, fcol - search); col <= std::min(map.width - 1, fcol + search);
         ++col) {
      // Stay on observed free. Carved/flooded cells past a SLAM seal look
      // safe on map but force through-wall plans on the live grid.
      if (!geometry.SafeViewpoint(col, row) || !observed.Free(col, row)) {
        continue;
      }
      const auto [x, y] = geometry.CellCenter(col, row);
      const double depth = DepthPastMouth(goal, x, y, robot_x, robot_y);
      const double lateral =
          std::abs((x - goal.frontier_x) * dir_y - (y - goal.frontier_y) * dir_x);
      if (lateral > .28 || depth < -.10 || depth > .55) {
        continue;
      }
      const double clearance = OccupiedClearanceAt(map, col, row);
      if (clearance < kPlanningRadius()) {
        continue;
      }
      const double score = 5.0 * clearance - 2.0 * lateral + 0.4 * depth;
      if (score > best_score) {
        best_score = score;
        best_col = col;
        best_row = row;
      }
    }
  }
  if (best_col < 0) {
    return;
  }
  const auto [x, y] = geometry.CellCenter(best_col, best_row);
  goal.x = x;
  goal.y = y;
  goal.yaw = std::atan2(dir_y, dir_x);
}

}  // namespace
std::vector<ExplorationGoal> PlaceMouthGoals(std::vector<ExplorationGoal> mouths,
                                             const ExplorationGrid& map,
                                             const ExplorationGrid& observation,
                                             double footprint_clearance, double robot_x,
                                             double robot_y) {
  const ObservationGridView observed(observation);
  for (auto& goal : mouths) {
    const double current_depth = DepthPastMouth(goal, goal.x, goal.y, robot_x, robot_y);
    if (current_depth >= kCloseStandoff()) {
      continue;
    }
    bool advanced = false;
    // Only pull the viewpoint into the room when that cell is already free
    // on the live observation map. Entry into still-unknown rooms is owned
    // by StartEntryProbe after the hall standoff is reached.
    if (auto probe = SelectEntryProbeAt(map, observation, footprint_clearance, goal, robot_x,
                                        robot_y, kCloseStandoff(), .5 * kNarrowMaxWidth())) {
      const int pcol = static_cast<int>(std::floor((probe->x - map.origin_x) / map.resolution));
      const int prow = static_cast<int>(std::floor((probe->y - map.origin_y) / map.resolution));
      if (observed.Free(pcol, prow) &&
          DepthPastMouth(goal, probe->x, probe->y, robot_x, robot_y) > current_depth) {
        goal.x = probe->x;
        goal.y = probe->y;
        goal.yaw = probe->yaw;
        advanced = true;
      }
    }
    if (!advanced || CorridorMouthAt(map, goal.frontier_x, goal.frontier_y)) {
      CenterMouthGoal(goal, map, observation, footprint_clearance, robot_x, robot_y);
    }
  }
  return mouths;
}

}  // namespace navigation2d::frontier_internal
