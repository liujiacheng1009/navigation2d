#include "navigation2d/exploration/frontier_explorer.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <optional>

int main() {
  navigation2d::FrontierExplorerConfig config;
  config.minimum_frontier_cells = 3;
  config.footprint_clearance = .1;
  config.required_frontier_observations = 1;
  navigation2d::FrontierExplorer explorer(config);
  navigation2d::ExplorationGrid grid;
  grid.width = grid.height = 20;
  grid.resolution = .1;
  grid.cells.assign(400, -1);
  for (int row = 4; row < 16; ++row)
    for (int col = 4; col < 16; ++col) grid.cells[row * grid.width + col] = 0;
  explorer.UpdateMap(grid);
  const auto goals = explorer.SelectGoals(1., 1.);
  assert(!goals.empty());
  assert(explorer.GoalStillFrontier(goals.front()));
  const int frontier_col = static_cast<int>(
      (goals.front().frontier_x - grid.origin_x) / grid.resolution);
  const int frontier_row = static_cast<int>(
      (goals.front().frontier_y - grid.origin_y) / grid.resolution);
  for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
    const int col = frontier_col + dx, row = frontier_row + dy;
    if (col >= 0 && row >= 0 && col < grid.width && row < grid.height)
      grid.cells[row * grid.width + col] = 0;
  }
  explorer.UpdateMap(grid);
  assert(!explorer.GoalStillFrontier(goals.front()));
  explorer.RecordAttempt(goals.front(), false);
  assert(explorer.failed_goals() == 1);
  const auto remaining = explorer.SelectGoals(1., 1.);
  for (const auto& goal : remaining)
    assert(goal.frontier_x != goals.front().frontier_x ||
           goal.frontier_y != goals.front().frontier_y);
  assert(!explorer.CompletionEligible(explorer.KnownCells()));

  // Frontier utility must use traversable grid distance, not straight-line
  // distance through unknown space. A disconnected observed island is never
  // returned as a candidate.
  navigation2d::ExplorationGrid split;
  split.width = 30; split.height = 20; split.resolution = .1;
  split.cells.assign(600, -1);
  for (int row = 3; row < 17; ++row) {
    for (int col = 2; col < 13; ++col) split.cells[row * split.width + col] = 0;
    for (int col = 17; col < 28; ++col) split.cells[row * split.width + col] = 0;
  }
  navigation2d::FrontierExplorer reachable_only(config);
  reachable_only.UpdateMap(split);
  const auto reachable_goals = reachable_only.SelectGoals(.7, 1.);
  assert(!reachable_goals.empty());
  for (const auto& goal : reachable_goals) assert(goal.x < 1.4);

  // One unknown cell in an open room is below minimum_frontier_cells, so it
  // stays out of the exploring tour. It must still become an approach goal.
  // The robot reaches it through ordinary open space; it does not need a
  // narrow-corridor fallback. Three adjacent unknown cells expand into a
  // 7-cell frontier and already produced a normal goal before this change.
  navigation2d::FrontierExplorerConfig leftover_config;
  leftover_config.minimum_frontier_cells = 6;
  leftover_config.footprint_clearance = .40;
  leftover_config.minimum_standoff = .55;
  leftover_config.maximum_standoff = 1.10;
  leftover_config.required_frontier_observations = 1;
  navigation2d::ExplorationGrid leftover;
  leftover.width = leftover.height = 30;
  leftover.resolution = .1;
  leftover.cells.assign(900, 100);
  for (int row = 4; row < 26; ++row)
    for (int col = 4; col < 26; ++col) leftover.cells[row * leftover.width + col] = 0;
  leftover.cells[24 * leftover.width + 24] = -1;
  navigation2d::FrontierExplorer leftover_explorer(leftover_config);
  leftover_explorer.UpdateMap(leftover);
  assert(leftover_explorer.SelectGoals(.8, .8).empty());
  const auto leftover_goal = leftover_explorer.SelectLeftoverApproach(.8, .8);
  assert(leftover_goal.has_value());
  assert(leftover_goal->leftover_approach);
  assert(leftover_explorer.raw_frontier_cells() > 0);

  // A 0.45 m pinch is wide enough for the 0.29 m chassis. The observation
  // pose has to sit on the far side of that pinch, beside the unknown
  // boundary. After that look succeeds, the same mouth must not be queued
  // again.
  navigation2d::FrontierExplorerConfig pinch_config;
  pinch_config.minimum_frontier_cells = 6;
  pinch_config.footprint_clearance = .22;
  pinch_config.minimum_standoff = .10;
  pinch_config.maximum_standoff = 1.10;
  pinch_config.blacklist_radius = 1.20;
  pinch_config.required_frontier_observations = 1;
  navigation2d::ExplorationGrid pinch;
  pinch.width = 80;
  pinch.height = 40;
  pinch.resolution = .05;
  pinch.cells.assign(static_cast<std::size_t>(pinch.width * pinch.height), 100);
  const auto fill = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row)
      for (int col = col0; col < col1; ++col)
        pinch.cells[static_cast<std::size_t>(row * pinch.width + col)] = value;
  };
  fill(4, 28, 14, 27, 0);
  fill(28, 36, 16, 25, 0);
  fill(36, 62, 8, 33, 0);
  fill(62, 70, 8, 33, -1);
  navigation2d::FrontierExplorer pinch_explorer(pinch_config);
  pinch_explorer.UpdateMap(pinch);
  const double robot_x = (8 + .5) * pinch.resolution;
  const double robot_y = (20 + .5) * pinch.resolution;
  const auto pinch_goals = pinch_explorer.SelectGoals(robot_x, robot_y);
  assert(!pinch_goals.empty());
  const double pinch_end = (36 + .5) * pinch.resolution;
  for (const auto& goal : pinch_goals) assert(goal.x > pinch_end);
  const auto first_look = pinch_goals.front();
  // The pose is the last safe cell beside the unknown boundary, not a
  // standoff back in the hall in front of the pinch.
  assert(std::hypot(first_look.x - first_look.frontier_x,
                    first_look.y - first_look.frontier_y) < .25);
  pinch_explorer.RecordAttempt(first_look, true);
  const auto second_look = pinch_explorer.SelectGoals(robot_x, robot_y);
  for (const auto& goal : second_look) {
    assert(goal.x > pinch_end);
    assert(std::hypot(goal.x - first_look.x, goal.y - first_look.y) >= .60);
  }

  // The hall is wide. The doorway is only partly observed: a strip beside one
  // jamb is free and the rest of the gap, plus the room beyond, is unknown.
  // The existing corridor rule completes that gap so the viewpoint sits on
  // the centerline. The clearance stays 0.22 m. A narrower gap, and the room
  // past the doorway, stay unknown.
  navigation2d::FrontierExplorerConfig door_config;
  door_config.minimum_frontier_cells = 6;
  door_config.footprint_clearance = .22;
  door_config.minimum_standoff = .10;
  door_config.maximum_standoff = 1.10;
  door_config.required_frontier_observations = 1;
  navigation2d::ExplorationGrid door;
  door.width = 80;
  door.height = 56;
  door.resolution = .05;
  door.cells.assign(static_cast<std::size_t>(door.width * door.height), 100);
  const auto paint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row)
      for (int col = col0; col < col1; ++col)
        door.cells[static_cast<std::size_t>(row * door.width + col)] = value;
  };
  paint(2, 28, 4, 48, 0);
  paint(28, 48, 18, 32, -1);
  paint(28, 29, 18, 22, 0);
  paint(48, 74, 4, 48, -1);
  // 0.30 m from wall to wall, below the corridor the chassis can enter.
  paint(60, 63, 50, 54, 0);
  paint(63, 66, 50, 54, -1);
  navigation2d::OpenPassableOpenings(&door);
  assert(door.cells[25 * door.width + 36] == 0);
  assert(door.cells[25 * door.width + 48] == -1);
  assert(door.cells[52 * door.width + 64] == -1);
  navigation2d::FrontierExplorer door_explorer(door_config);
  door_explorer.UpdateMap(door);
  const auto door_goals = door_explorer.SelectGoals(
      (8 + .5) * door.resolution, (25 + .5) * door.resolution);
  assert(!door_goals.empty());
  const double door_mouth = (28 + .5) * door.resolution;
  bool on_centerline = false;
  for (const auto& goal : door_goals) {
    if (goal.x < door_mouth) continue;
    on_centerline = true;
    assert(std::hypot(goal.x - goal.frontier_x, goal.y - goal.frontier_y) < .25);
    double nearest_occupied = 1.;
    for (int row = 0; row < door.height; ++row) {
      for (int col = 0; col < door.width; ++col) {
        if (door.cells[static_cast<std::size_t>(row * door.width + col)] <= 0) continue;
        const double x = (col + .5) * door.resolution;
        const double y = (row + .5) * door.resolution;
        nearest_occupied = std::min(nearest_occupied, std::hypot(goal.x - x, goal.y - y));
      }
    }
    assert(nearest_occupied >= .22);
  }
  assert(on_centerline);
}
