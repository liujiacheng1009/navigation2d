#include "navigation2d/exploration/frontier_explorer.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

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

  // Vacuum coverage: a nearby low-gain doorway must lead the tour before a
  // far high-gain wall, even though NBV would prefer the distant blob.
  navigation2d::FrontierExplorerConfig tour_config;
  tour_config.minimum_frontier_cells = 6;
  tour_config.footprint_clearance = navigation2d::kPlanningRadius;
  tour_config.minimum_standoff = .10;
  tour_config.maximum_standoff = 1.10;
  tour_config.required_frontier_observations = 1;
  navigation2d::ExplorationGrid tour_map;
  tour_map.width = 80;
  tour_map.height = 40;
  tour_map.resolution = .05;
  tour_map.cells.assign(static_cast<std::size_t>(tour_map.width * tour_map.height), 100);
  const auto tour_fill = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row)
      for (int col = col0; col < col1; ++col)
        tour_map.cells[static_cast<std::size_t>(row * tour_map.width + col)] = value;
  };
  tour_fill(4, 50, 12, 28, 0);
  tour_fill(10, 16, 28, 34, -1);
  tour_fill(50, 76, 8, 32, -1);
  navigation2d::FrontierExplorer tour_explorer(tour_config);
  tour_explorer.UpdateMap(tour_map);
  const double tour_robot_x = (12 + .5) * tour_map.resolution;
  const double tour_robot_y = (20 + .5) * tour_map.resolution;
  const auto tour = tour_explorer.BuildTour(tour_robot_x, tour_robot_y);
  assert(!tour.empty());
  assert(tour.front().x < 1.5);

  // Nearby open-boundary expansion must not beat a farther unfinished kitchen
  // doorway. Room/alcove throats lead the tour before hallway wall crawling.
  navigation2d::ExplorationGrid kitchen_first;
  kitchen_first.width = 100;
  kitchen_first.height = 50;
  kitchen_first.resolution = .05;
  kitchen_first.cells.assign(
      static_cast<std::size_t>(kitchen_first.width * kitchen_first.height), 100);
  const auto kfirst = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row)
      for (int col = col0; col < col1; ++col)
        kitchen_first.cells[static_cast<std::size_t>(row * kitchen_first.width + col)] =
            value;
  };
  kfirst(4, 70, 18, 34, 0);          // hallway
  kfirst(4, 22, 34, 46, -1);         // nearby wide open expansion (not a door)
  kfirst(70, 96, 10, 40, -1);        // kitchen room
  kfirst(68, 70, 20, 32, 0);         // 0.60 m throat into kitchen
  kfirst(68, 70, 20, 24, 0);
  navigation2d::FrontierExplorer kitchen_first_explorer(tour_config);
  kitchen_first_explorer.UpdateMap(kitchen_first);
  const double kfx = (20 + .5) * kitchen_first.resolution;
  const double kfy = (26 + .5) * kitchen_first.resolution;
  const auto kitchen_first_tour = kitchen_first_explorer.BuildTour(kfx, kfy);
  assert(!kitchen_first_tour.empty());
  // Dispatch commits the kitchen mouth; later tours stay on that region
  // until its executable observations are exhausted.
  kitchen_first_explorer.CommitRegion(kitchen_first_tour.front());
  const auto after_enter = kitchen_first_explorer.BuildTour(kfx, kfy);
  if (!after_enter.empty() && kitchen_first_tour.front().region_id != 0) {
    assert(after_enter.front().region_id == kitchen_first_tour.front().region_id ||
           kitchen_first_explorer.GoalInActiveRegion(after_enter.front()));
  }

  // One task per mouth: tour stops for distinct mouths stay well separated.
  for (std::size_t i = 0; i < tour.size(); ++i) {
    for (std::size_t j = i + 1; j < tour.size(); ++j) {
      assert(std::hypot(tour[i].x - tour[j].x, tour[i].y - tour[j].y) > 1.15 ||
             std::hypot(tour[i].frontier_x - tour[j].frontier_x,
                        tour[i].frontier_y - tour[j].frontier_y) > 1.15);
    }
  }

  // Fragmented frontier cells along one alcove mouth must collapse to a single
  // tour stop — not a pile of micro-cluster T-labels.
  navigation2d::ExplorationGrid alcove;
  alcove.width = 60;
  alcove.height = 40;
  alcove.resolution = .05;
  alcove.cells.assign(static_cast<std::size_t>(alcove.width * alcove.height), 100);
  const auto alcove_fill = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row)
      for (int col = col0; col < col1; ++col)
        alcove.cells[static_cast<std::size_t>(row * alcove.width + col)] = value;
  };
  alcove_fill(4, 40, 8, 32, 0);
  // Jagged unknown pocket (~1 m wide) that splits into several frontier clusters.
  alcove_fill(18, 28, 32, 38, -1);
  alcove.cells[static_cast<std::size_t>(31 * alcove.width + 20)] = -1;
  alcove.cells[static_cast<std::size_t>(31 * alcove.width + 22)] = -1;
  alcove.cells[static_cast<std::size_t>(31 * alcove.width + 25)] = -1;
  navigation2d::FrontierExplorer alcove_explorer(tour_config);
  alcove_explorer.UpdateMap(alcove);
  const auto alcove_tour = alcove_explorer.BuildTour(
      (20 + .5) * alcove.resolution, (18 + .5) * alcove.resolution);
  assert(!alcove_tour.empty());
  assert(alcove_tour.size() == 1);

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
  // Mature map (>= 10000 known cells): a 1-cell speck is not a doorway and
  // must stay out of SelectGoals, while SelectLeftoverApproach still finds it.
  navigation2d::ExplorationGrid leftover;
  leftover.width = leftover.height = 110;
  leftover.resolution = .1;
  leftover.cells.assign(static_cast<std::size_t>(leftover.width * leftover.height), 100);
  for (int row = 2; row < 108; ++row)
    for (int col = 2; col < 108; ++col) leftover.cells[row * leftover.width + col] = 0;
  leftover.cells[100 * leftover.width + 100] = -1;
  navigation2d::FrontierExplorer leftover_explorer(leftover_config);
  leftover_explorer.UpdateMap(leftover);
  assert(leftover_explorer.KnownCells() >= 10000);
  assert(leftover_explorer.SelectGoals(5., 5.).empty());
  const auto leftover_goal = leftover_explorer.SelectLeftoverApproach(5., 5.);
  assert(leftover_goal.has_value());
  assert(leftover_goal->leftover_approach);
  assert(leftover_explorer.raw_frontier_cells() > 0);
  // A temporary failure cool must not hide the unfinished tip from leftover
  // completing — that was returning PARTIAL with white alcoves still visible.
  leftover_explorer.RecordAttempt(*leftover_goal, false);
  assert(leftover_explorer.SelectGoals(5., 5.).empty());
  const auto leftover_after_fail =
      leftover_explorer.SelectLeftoverApproach(5., 5.);
  assert(leftover_after_fail.has_value());
  leftover_explorer.ClearFailedBlacklist();
  assert(leftover_explorer.SelectLeftoverApproaches(5., 5., 3).size() >= 1);
  // Wide pocket cool (tip-leave) must suppress leftover re-entry into a wall
  // tip — narrow failure cools above must not.
  leftover_explorer.CoolRegion(leftover_goal->frontier_x, leftover_goal->frontier_y,
                               1.80, 3000u);
  assert(!leftover_explorer.SelectLeftoverApproach(5., 5.).has_value());

  // Mature open room with a >=minimum_frontier_cells residual strip against a
  // furniture wall: SelectGoals must not place a tour GOAL in already-explored
  // free space (the red-box junction thrash). Completing leftovers may still
  // see the fragment.
  navigation2d::ExplorationGrid residual = leftover;
  for (int row = 40; row < 49; ++row)
    residual.cells[static_cast<std::size_t>(row * residual.width + 20)] = -1;
  for (int row = 40; row < 49; ++row)
    residual.cells[static_cast<std::size_t>(row * residual.width + 19)] = 100;
  navigation2d::FrontierExplorer residual_explorer(leftover_config);
  residual_explorer.UpdateMap(residual);
  assert(residual_explorer.KnownCells() >= 10000);
  assert(residual_explorer.raw_frontier_cells() >= 6);
  const auto residual_tour = residual_explorer.SelectGoals(5., 5.);
  assert(residual_tour.empty());
  assert(residual_explorer.SelectLeftoverApproach(5., 5.).has_value());

  // A 0.45 m pinch is wide enough for the 0.29 m chassis. The observation
  // pose has to sit on the far side of that pinch, beside the unknown
  // boundary. After that look succeeds, the same mouth must not be queued
  // again.
  navigation2d::FrontierExplorerConfig pinch_config;
  pinch_config.minimum_frontier_cells = 6;
  pinch_config.footprint_clearance = navigation2d::kPlanningRadius;
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
                    first_look.y - first_look.frontier_y) <= pinch_config.maximum_standoff);
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
  door_config.footprint_clearance = navigation2d::kPlanningRadius;
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
  // Without OpenPassable flood, unknown beyond the free speck stays unknown.
  assert(door.cells[25 * door.width + 36] == -1);
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
    assert(std::hypot(goal.x - goal.frontier_x, goal.y - goal.frontier_y) <= door_config.maximum_standoff);
    double nearest_occupied = 1.;
    for (int row = 0; row < door.height; ++row) {
      for (int col = 0; col < door.width; ++col) {
        if (door.cells[static_cast<std::size_t>(row * door.width + col)] <= 0) continue;
        const double x = (col + .5) * door.resolution;
        const double y = (row + .5) * door.resolution;
        nearest_occupied = std::min(nearest_occupied, std::hypot(goal.x - x, goal.y - y));
      }
    }
    assert(nearest_occupied >= navigation2d::kPlanningRadius);
  }
  assert(on_centerline);

  // A doorway arrival is only the observation-side pose. When a partially
  // observed narrow opening is safely opened in the private planning grid,
  // the explorer must be able to produce a reachable pose on the far side of
  // the frontier for the bounded entry probe.
  navigation2d::ExplorationGrid entry;
  entry.width = 50;
  entry.height = 30;
  entry.resolution = .05;
  entry.cells.assign(static_cast<std::size_t>(entry.width * entry.height), 100);
  for (int row = 4; row < 26; ++row)
    for (int col = 2; col < 16; ++col)
      entry.cells[static_cast<std::size_t>(row * entry.width + col)] = 0;
  for (int row = 4; row < 26; ++row)
    for (int col = 21; col < 29; ++col)
      entry.cells[static_cast<std::size_t>(row * entry.width + col)] = 0;
  for (int row = 8; row < 22; ++row)
    for (int col = 16; col < 21; ++col)
      entry.cells[static_cast<std::size_t>(row * entry.width + col)] = -1;
  navigation2d::FrontierExplorer entry_explorer(door_config);
  entry_explorer.UpdateMap(entry);
  // The opening is reachable in the private planning graph, while it is
  // still unknown in the observation map; completion must account for it.
  assert(entry_explorer.ReachableUnknownCells(.4, .7) > 0);
  const auto entry_goals = entry_explorer.SelectGoals(.4, .7);
  assert(!entry_goals.empty());
  // OneGoalPerMouth prefers an entry-side pose when the planning graph opened
  // the gap: the task point must lie past the frontier toward the room.
  const auto& entry_goal = entry_goals.front();
  const double entry_depth =
      ((entry_goal.x - entry_goal.frontier_x) *
           (entry_goal.frontier_x - .4) +
       (entry_goal.y - entry_goal.frontier_y) *
           (entry_goal.frontier_y - .7)) /
      std::hypot(entry_goal.frontier_x - .4, entry_goal.frontier_y - .7);
  assert(entry_depth >= .15);
  // SelectEntryProbe still works from a hallway-side standoff for the
  // runtime entry state machine.
  navigation2d::ExplorationGoal hallway_standoff = entry_goal;
  hallway_standoff.x = entry_goal.frontier_x - .45;
  hallway_standoff.y = entry_goal.frontier_y;
  const auto entry_probe = entry_explorer.SelectEntryProbe(
      hallway_standoff, .4, .7, .20, .70);
  assert(entry_probe.has_value());
  const double entry_dx = entry_probe->x - hallway_standoff.frontier_x;
  const double entry_dy = entry_probe->y - hallway_standoff.frontier_y;
  const double entry_direction_x = hallway_standoff.frontier_x - hallway_standoff.x;
  const double entry_direction_y = hallway_standoff.frontier_y - hallway_standoff.y;
  const double entry_direction_length =
      std::hypot(entry_direction_x, entry_direction_y);
  assert((entry_dx * entry_direction_x + entry_dy * entry_direction_y) /
             entry_direction_length >= .20);
  // Probe may sit on planning free past the frontier when the throat is open.
  assert(!entry_explorer.RawFree(
      static_cast<int>(std::floor(
          (entry_probe->x - entry.origin_x) / entry.resolution)),
      static_cast<int>(std::floor(
          (entry_probe->y - entry.origin_y) / entry.resolution))) ||
         entry_probe->x > hallway_standoff.frontier_x - 1e-6);

  // Opening a partially observed passage is a planning-only transformation.
  // The mapper's unknown boundary must remain visible to completion logic and
  // the reported known area must not grow just because a narrow gap was
  // opened in the private navigation graph.
  navigation2d::ExplorationGrid raw_opening;
  raw_opening.width = raw_opening.height = 40;
  raw_opening.resolution = .05;
  raw_opening.cells.assign(static_cast<std::size_t>(raw_opening.width * raw_opening.height), 100);
  for (int row = 10; row < 30; ++row)
    for (int col = 4; col < 22; ++col) raw_opening.cells[row * raw_opening.width + col] = 0;
  for (int row = 16; row < 26; ++row)
    for (int col = 22; col < 30; ++col) raw_opening.cells[row * raw_opening.width + col] = -1;
  const auto raw_known = static_cast<std::size_t>(
      std::count_if(raw_opening.cells.begin(), raw_opening.cells.end(),
                    [](std::int8_t value) { return value >= 0; }));
  navigation2d::FrontierExplorer raw_explorer(door_config);
  raw_explorer.UpdateMap(raw_opening);
  assert(raw_explorer.KnownCells() == raw_known);
  assert(raw_explorer.unresolved_frontier_cells() > 0);
  assert(!raw_explorer.SelectGoals(.5, .95).empty());

  // Two doorways whose mouths are ~2.0 m apart (beyond the 1.5 m mouth merge).
  // Observing one cools that mouth only.
  navigation2d::FrontierExplorerConfig pair_config;
  pair_config.minimum_frontier_cells = 6;
  pair_config.footprint_clearance = navigation2d::kPlanningRadius;
  pair_config.minimum_standoff = .10;
  pair_config.maximum_standoff = 1.10;
  pair_config.blacklist_radius = 1.20;
  pair_config.required_frontier_observations = 1;
  navigation2d::ExplorationGrid pair;
  pair.width = 60;
  pair.height = 64;
  pair.resolution = .05;
  pair.cells.assign(static_cast<std::size_t>(pair.width * pair.height), 100);
  const auto pair_paint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row)
      for (int col = col0; col < col1; ++col)
        pair.cells[static_cast<std::size_t>(row * pair.width + col)] = value;
  };
  pair_paint(2, 22, 2, 62, 0);
  pair_paint(22, 50, 6, 16, -1);
  pair_paint(22, 50, 46, 56, -1);
  navigation2d::FrontierExplorer pair_explorer(pair_config);
  pair_explorer.UpdateMap(pair);
  const auto pair_goals = pair_explorer.SelectGoals(
      (8 + .5) * pair.resolution, (11 + .5) * pair.resolution);
  const navigation2d::ExplorationGoal* lower = nullptr;
  const navigation2d::ExplorationGoal* upper = nullptr;
  for (const auto& goal : pair_goals) {
    if (goal.frontier_y < (32 + .5) * pair.resolution) {
      if (lower == nullptr || goal.frontier_cells > lower->frontier_cells) lower = &goal;
    } else if (upper == nullptr || goal.frontier_cells > upper->frontier_cells) {
      upper = &goal;
    }
  }
  assert(lower != nullptr && upper != nullptr);
  assert(std::abs(lower->frontier_y - upper->frontier_y) > 1.5);
  assert(std::abs(lower->frontier_y - upper->frontier_y) < 2.5);
  pair_explorer.RecordAttempt(*lower, true);
  const auto upper_goals = pair_explorer.SelectGoals(
      (8 + .5) * pair.resolution, (51 + .5) * pair.resolution);
  bool upper_remains = false;
  for (const auto& goal : upper_goals) {
    if (std::hypot(goal.frontier_x - upper->frontier_x,
                   goal.frontier_y - upper->frontier_y) < .40)
      upper_remains = true;
  }
  assert(upper_remains);
  // A failed standoff must not suppress the entire boundary or its adjacent
  // doorway. The old span+0.25 radius could cover both rooms at once.
  navigation2d::FrontierExplorer failed_pair(pair_config);
  failed_pair.UpdateMap(pair);
  auto failed = *lower;
  failed.x -= 1.5;
  failed_pair.RecordAttempt(failed, false);
  const auto alternatives = failed_pair.SelectGoals(
      (8 + .5) * pair.resolution, (11 + .5) * pair.resolution);
  bool lower_available = false, upper_available = false;
  for (const auto& goal : alternatives) {
    lower_available |= std::hypot(goal.frontier_x - lower->frontier_x,
                                   goal.frontier_y - lower->frontier_y) < .4;
    upper_available |= std::hypot(goal.frontier_x - upper->frontier_x,
                                   goal.frontier_y - upper->frontier_y) < .4;
  }
  assert(lower_available && upper_available);

  // Junction: three nearby mouths + one far room. Tour must lead with a near
  // mouth; a far active goal must be preemptable until local mouths are gone.
  navigation2d::FrontierExplorerConfig junction_config;
  junction_config.minimum_frontier_cells = 6;
  junction_config.footprint_clearance = navigation2d::kPlanningRadius;
  junction_config.minimum_standoff = .10;
  junction_config.maximum_standoff = 1.10;
  junction_config.required_frontier_observations = 1;
  navigation2d::ExplorationGrid junction;
  junction.width = 120;
  junction.height = 70;
  junction.resolution = .05;
  junction.cells.assign(static_cast<std::size_t>(junction.width * junction.height), 100);
  const auto jpaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row)
      for (int col = col0; col < col1; ++col)
        junction.cells[static_cast<std::size_t>(row * junction.width + col)] = value;
  };
  jpaint(25, 70, 28, 42, 0);
  jpaint(36, 48, 8, 62, 0);
  jpaint(6, 25, 28, 42, -1);
  jpaint(36, 48, 62, 68, -1);
  jpaint(36, 48, 2, 8, -1);
  jpaint(100, 118, 28, 42, -1);
  navigation2d::FrontierExplorer junction_explorer(junction_config);
  junction_explorer.UpdateMap(junction);
  const double jx = (40 + .5) * junction.resolution;
  const double jy = (35 + .5) * junction.resolution;
  const auto junction_tour = junction_explorer.BuildTour(jx, jy);
  assert(!junction_tour.empty());
  assert(junction_tour.front().x < 4.5);
  navigation2d::ExplorationGoal far_active;
  far_active.x = (105 + .5) * junction.resolution;
  far_active.y = (35 + .5) * junction.resolution;
  far_active.frontier_x = (100 + .5) * junction.resolution;
  far_active.frontier_y = (35 + .5) * junction.resolution;
  far_active.frontier_cells = 20;
  far_active.information_gain = 10.;
  assert(junction_explorer.ShouldPreemptForNearerMouth(jx, jy, far_active, 6.0));
  jpaint(6, 25, 28, 42, 0);
  jpaint(36, 48, 62, 68, 0);
  jpaint(36, 48, 2, 8, 0);
  junction_explorer.UpdateMap(junction);
  assert(!junction_explorer.ShouldPreemptForNearerMouth(jx, jy, far_active, 6.0));

  // Pinched alcove tip: frontier cells fail ClearAt while a centerline pose
  // behind them is still on the discovery graph. Those tips must become
  // candidates (approachable frontier), not vanish into candidates=0.
  navigation2d::ExplorationGrid tip;
  tip.width = 50;
  tip.height = 60;
  tip.resolution = .05;
  tip.cells.assign(static_cast<std::size_t>(tip.width * tip.height), 100);
  const auto tpaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row)
      for (int col = col0; col < col1; ++col)
        tip.cells[static_cast<std::size_t>(row * tip.width + col)] = value;
  };
  tpaint(8, 42, 4, 30, 0);           // open hall
  tpaint(19, 29, 30, 48, 0);         // 0.50 m north alcove (planning radius .22)
  tpaint(19, 29, 48, 56, -1);        // unfinished tip unknown
  // Thicken the tip free cells with near walls so ClearAt(.22) fails on the
  // frontier row itself but the approach row still clears.
  tpaint(19, 20, 46, 48, 100);
  tpaint(28, 29, 46, 48, 100);
  navigation2d::FrontierExplorerConfig tip_config = tour_config;
  tip_config.footprint_clearance = .30;
  tip_config.required_frontier_observations = 1;
  navigation2d::FrontierExplorer tip_explorer(tip_config);
  tip_explorer.UpdateMap(tip);
  const auto tip_goals = tip_explorer.SelectGoals(
      (25 + .5) * tip.resolution, (15 + .5) * tip.resolution);
  assert(!tip_goals.empty());
  assert(tip_explorer.candidate_frontier_goals() > 0);
  assert(tip_goals.front().frontier_y > 1.8);

  // Both-narrow vestibule (kitchen mouth): SelectGoals must still nominate a
  // candidate. The old XOR narrow_viewpoint treated both-narrow as open space
  // and left raw frontiers with zero candidates → early PARTIAL.
  navigation2d::ExplorationGrid vestibule;
  vestibule.width = 60;
  vestibule.height = 40;
  vestibule.resolution = .05;
  vestibule.cells.assign(static_cast<std::size_t>(vestibule.width * vestibule.height), 100);
  const auto vpaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row)
      for (int col = col0; col < col1; ++col)
        vestibule.cells[static_cast<std::size_t>(row * vestibule.width + col)] = value;
  };
  vpaint(4, 28, 14, 26, 0);          // hall
  vpaint(28, 40, 15, 25, 0);         // 0.60 m × 0.50 m vestibule (both-narrow)
  vpaint(40, 56, 10, 30, -1);        // room unknown
  navigation2d::FrontierExplorerConfig vestibule_config = tour_config;
  vestibule_config.footprint_clearance = .30;
  vestibule_config.required_frontier_observations = 1;
  navigation2d::FrontierExplorer vestibule_explorer(vestibule_config);
  vestibule_explorer.UpdateMap(vestibule);
  const auto vestibule_goals = vestibule_explorer.SelectGoals(
      (12 + .5) * vestibule.resolution, (20 + .5) * vestibule.resolution);
  assert(!vestibule_goals.empty());
  assert(vestibule_explorer.candidate_frontier_goals() > 0);

  // Physical 0.70 m kitchen door with production clearance (.30). Viewpoint
  // nomination must still place a task past the mouth into the kitchen; the
  // old corridor ClearAt(.30) rejected the centerline after jamb rasterization.
  navigation2d::FrontierExplorerConfig kitchen_config;
  kitchen_config.minimum_frontier_cells = 6;
  kitchen_config.footprint_clearance = .30;
  kitchen_config.minimum_standoff = .10;
  kitchen_config.maximum_standoff = 1.10;
  kitchen_config.required_frontier_observations = 1;
  navigation2d::ExplorationGrid kitchen;
  kitchen.width = 80;
  kitchen.height = 40;
  kitchen.resolution = .05;
  kitchen.cells.assign(static_cast<std::size_t>(kitchen.width * kitchen.height), 100);
  const auto kpaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row)
      for (int col = col0; col < col1; ++col)
        kitchen.cells[static_cast<std::size_t>(row * kitchen.width + col)] = value;
  };
  // Hall on the left; 0.70 m = 14-cell vertical doorway; kitchen unknown beyond.
  // the gap as a partly observed passable opening (same contract as door_test).
  kpaint(2, 28, 4, 36, 0);
  kpaint(28, 48, 13, 27, -1);   // 14 cells * 0.05 = 0.70 m
  kpaint(28, 30, 13, 20, 0);
  kpaint(48, 74, 4, 36, -1);
  assert(kitchen.cells[20 * kitchen.width + 36] == 0);
  navigation2d::FrontierExplorer kitchen_explorer(kitchen_config);
  kitchen_explorer.UpdateMap(kitchen);
  const double kx = (10 + .5) * kitchen.resolution;
  const double ky = (20 + .5) * kitchen.resolution;
  assert(kitchen_explorer.ReachableUnknownCells(kx, ky) > 0);
  const auto kitchen_goals = kitchen_explorer.SelectGoals(kx, ky);
  assert(!kitchen_goals.empty());
  const auto& kg = kitchen_goals.front();
  const double kitchen_depth =
      ((kg.x - kg.frontier_x) * (kg.frontier_x - kx) +
       (kg.y - kg.frontier_y) * (kg.frontier_y - ky)) /
      std::hypot(kg.frontier_x - kx, kg.frontier_y - ky);
  assert(kitchen_depth >= .10);
  navigation2d::ExplorationGoal hall_standoff = kg;
  hall_standoff.x = kg.frontier_x - .45;
  hall_standoff.y = kg.frontier_y;
  const auto kitchen_probe = kitchen_explorer.SelectEntryProbe(
      hall_standoff, kx, ky, .20, .70);
  assert(kitchen_probe.has_value());
  assert(kitchen_probe->x > kg.frontier_x - .05);
  // Doorway probe / standoff must sit mid-gap, not glued to a jamb.
  {
    const int pcol = static_cast<int>(std::floor(
        (kitchen_probe->x - kitchen.origin_x) / kitchen.resolution));
    const int prow = static_cast<int>(std::floor(
        (kitchen_probe->y - kitchen.origin_y) / kitchen.resolution));
    double nearest = 10.;
    for (int row = 0; row < kitchen.height; ++row) {
      for (int col = 0; col < kitchen.width; ++col) {
        if (kitchen.cells[static_cast<std::size_t>(row * kitchen.width + col)] <= 0)
          continue;
        const double ox = (col + .5) * kitchen.resolution;
        const double oy = (row + .5) * kitchen.resolution;
        nearest = std::min(nearest, std::hypot(kitchen_probe->x - ox,
                                               kitchen_probe->y - oy));
      }
    }
    assert(nearest + 1e-9 >= .14);
    (void)pcol;
    (void)prow;
  }
  // Probe must sit on observed free, not flooded unknown past the map.
  {
    const int pc = static_cast<int>(
        std::floor(kitchen_probe->x / kitchen.resolution));
    const int pr = static_cast<int>(
        std::floor(kitchen_probe->y / kitchen.resolution));
    assert(kitchen.cells[static_cast<std::size_t>(pr) * kitchen.width + pc] ==
           0);
  }
  // Lateral hall standoff (beside the jamb): old viewpoint→frontier axis
  // walked along the wall; unknown-centroid must still send the probe into
  // the kitchen (+x).
  navigation2d::ExplorationGoal lateral = kg;
  lateral.x = kg.frontier_x;
  lateral.y = kg.frontier_y - .55;
  const auto lateral_probe = kitchen_explorer.SelectEntryProbe(
      lateral, kx, ky, .20, .70);
  assert(lateral_probe.has_value());
  assert(lateral_probe->x > kg.frontier_x + .10);

  // Pinched kitchen mouth: free speck in a 0.20 m unknown throat. Without
  // OpenPassable flood the speck stays free and unknown past it stays unknown.
  navigation2d::ExplorationGrid pinched;
  pinched.width = 60;
  pinched.height = 40;
  pinched.resolution = .05;
  pinched.cells.assign(static_cast<std::size_t>(pinched.width * pinched.height), 100);
  const auto ppaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row)
      for (int col = col0; col < col1; ++col)
        pinched.cells[static_cast<std::size_t>(row * pinched.width + col)] = value;
  };
  ppaint(2, 28, 4, 36, 0);           // hall
  ppaint(28, 48, 18, 22, -1);        // 0.20 m unknown throat (4 cells)
  ppaint(28, 30, 18, 20, 0);         // observed free speck in throat
  ppaint(48, 58, 4, 36, -1);         // kitchen unknown
  assert(pinched.cells[19 * pinched.width + 29] == 0);
  assert(pinched.cells[20 * pinched.width + 40] == -1);

  // Door-width occupied bar across a kitchen mouth: SLAM occupied is lethal.
  // PathCuts rejects the entry chord until the map clears the seal.
  navigation2d::ExplorationGrid barred;
  barred.width = 60;
  barred.height = 40;
  barred.resolution = .05;
  barred.cells.assign(static_cast<std::size_t>(barred.width * barred.height), 100);
  const auto bpaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row)
      for (int col = col0; col < col1; ++col)
        barred.cells[static_cast<std::size_t>(row * barred.width + col)] = value;
  };
  bpaint(2, 58, 4, 18, 0);     // south hall free
  bpaint(2, 58, 22, 36, -1);   // north kitchen unknown
  bpaint(20, 34, 18, 22, 0);   // 0.70 m mouth
  bpaint(20, 34, 20, 21, 100); // 1-cell occupied bar sealing the mouth
  {
    const double rx = (27 + .5) * barred.resolution;
    const double ry = (17 + .5) * barred.resolution;
    const std::vector<std::pair<double, double>> entry = {
        {rx, ry},
        {rx, (24 + .5) * barred.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(barred, entry, rx, ry));
  }
  assert(barred.cells[20 * barred.width + 27] == 100);
  assert(barred.cells[24 * barred.width + 27] == -1);

  // free↔free door-width residual seal: PathCuts still rejects through-bar.
  {
    navigation2d::ExplorationGrid both_free;
    both_free.width = 60;
    both_free.height = 40;
    both_free.resolution = .05;
    both_free.cells.assign(
        static_cast<std::size_t>(both_free.width * both_free.height), 100);
    const auto fpaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row)
        for (int col = col0; col < col1; ++col)
          both_free.cells[static_cast<std::size_t>(row * both_free.width + col)] =
              value;
    };
    fpaint(2, 58, 4, 18, 0);      // south hall
    fpaint(2, 58, 22, 36, 0);     // north kitchen now explored
    fpaint(20, 34, 18, 22, 0);    // 0.70 m mouth vestibule
    fpaint(20, 34, 20, 21, 100);  // occupied bar
    fpaint(26, 29, 22, 24, -1);
    const double rx = (27 + .5) * both_free.resolution;
    const double ry = (17 + .5) * both_free.resolution;
    const std::vector<std::pair<double, double>> through_bar = {
        {rx, ry},
        {rx, (24 + .5) * both_free.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(both_free, through_bar, rx, ry));
  }

  // Vestibule: both free runs are door-width (~0.50 m). SafeViewpoint must
  // still accept the center with planning radius (.22); the old XOR treated
  // both-narrow as open space and demanded footprint_clearance (.30).
  {
    navigation2d::FrontierExplorerConfig vestibule_config;
    vestibule_config.footprint_clearance = .30;
    vestibule_config.minimum_frontier_cells = 6;
    vestibule_config.minimum_standoff = .08;
    vestibule_config.maximum_standoff = 1.10;
    vestibule_config.required_frontier_observations = 1;
    navigation2d::ExplorationGrid vestibule;
    vestibule.width = 40;
    vestibule.height = 40;
    vestibule.resolution = .05;
    vestibule.cells.assign(static_cast<std::size_t>(vestibule.width * vestibule.height), 100);
    for (int row = 10; row < 20; ++row)
      for (int col = 10; col < 30; ++col)
        vestibule.cells[static_cast<std::size_t>(row * vestibule.width + col)] = 0;
    // 0.50 m × 0.50 m kitchen vestibule past a hall, then unknown room.
    for (int row = 20; row < 30; ++row)
      for (int col = 15; col < 25; ++col)
        vestibule.cells[static_cast<std::size_t>(row * vestibule.width + col)] = 0;
    for (int row = 30; row < 38; ++row)
      for (int col = 15; col < 25; ++col)
        vestibule.cells[static_cast<std::size_t>(row * vestibule.width + col)] = -1;
    navigation2d::FrontierExplorer vestibule_explorer(vestibule_config);
    vestibule_explorer.UpdateMap(vestibule);
    const double robot_x = (20 + .5) * vestibule.resolution;
    const double robot_y = (14 + .5) * vestibule.resolution;
    const auto goals = vestibule_explorer.SelectGoals(robot_x, robot_y);
    assert(!goals.empty());
    const auto tour = vestibule_explorer.BuildTour(robot_x, robot_y);
    assert(!tour.empty());
  }

  // Occupied speck on a free hall row stays occupied; unknown beyond is not
  // flooded into free for planning.
  navigation2d::ExplorationGrid wide_hall;
  wide_hall.width = 80;
  wide_hall.height = 40;
  wide_hall.resolution = .05;
  wide_hall.cells.assign(static_cast<std::size_t>(wide_hall.width * wide_hall.height), 100);
  const auto wpaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
    for (int row = row0; row < row1; ++row)
      for (int col = col0; col < col1; ++col)
        wide_hall.cells[static_cast<std::size_t>(row * wide_hall.width + col)] = value;
  };
  wpaint(2, 78, 4, 21, 0);      // open hall including the bar row
  wpaint(2, 78, 21, 36, -1);    // room unknown immediately past the bar
  wpaint(30, 44, 20, 21, 100);  // occupied speck on the hall row
  assert(wide_hall.cells[20 * wide_hall.width + 37] == 100);
  assert(wide_hall.cells[24 * wide_hall.width + 37] == -1);

  // Structural free↔free wall stays occupied (no seal carve).
  {
    navigation2d::ExplorationGrid structural;
    structural.width = 60;
    structural.height = 40;
    structural.resolution = .05;
    structural.cells.assign(
        static_cast<std::size_t>(structural.width * structural.height), 100);
    const auto spaint = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row)
        for (int col = col0; col < col1; ++col)
          structural.cells[static_cast<std::size_t>(row * structural.width + col)] =
              value;
    };
    spaint(2, 58, 4, 18, 0);      // south room free
    spaint(2, 58, 19, 30, 0);     // north room free (wall at row 18)
    spaint(2, 58, 18, 19, 100);   // continuous structural wall
    spaint(2, 58, 30, 38, -1);    // unknown only deep past free north room
    assert(structural.cells[18 * structural.width + 30] == 100);
  }

  // Tour goals must stay on observed free — unknown exterior must not become
  // a planning GOAL outside the mapped room.
  {
    navigation2d::FrontierExplorerConfig flood_config;
    flood_config.minimum_frontier_cells = 4;
    flood_config.footprint_clearance = .30;
    flood_config.minimum_standoff = .20;
    flood_config.maximum_standoff = 1.20;
    flood_config.required_frontier_observations = 1;
    navigation2d::ExplorationGrid flooded;
    flooded.width = 60;
    flooded.height = 40;
    flooded.resolution = .05;
    flooded.cells.assign(
        static_cast<std::size_t>(flooded.width * flooded.height), 100);
    for (int row = 8; row < 32; ++row)
      for (int col = 8; col < 36; ++col)
        flooded.cells[static_cast<std::size_t>(row * flooded.width + col)] = 0;
    // Narrow free nick in the east wall, then a band of unknown outside.
    for (int row = 18; row < 22; ++row)
      flooded.cells[static_cast<std::size_t>(row * flooded.width + 36)] = 0;
    for (int row = 10; row < 30; ++row)
      for (int col = 37; col < 55; ++col)
        flooded.cells[static_cast<std::size_t>(row * flooded.width + col)] = -1;
    navigation2d::FrontierExplorer flood_explorer(flood_config);
    flood_explorer.UpdateMap(flooded);
    const double fx = (20 + .5) * flooded.resolution;
    const double fy = (20 + .5) * flooded.resolution;
    const auto flood_goals = flood_explorer.SelectGoals(fx, fy);
    for (const auto& goal : flood_goals) {
      const int gcol = static_cast<int>(
          std::floor((goal.x - flooded.origin_x) / flooded.resolution));
      const int grow = static_cast<int>(
          std::floor((goal.y - flooded.origin_y) / flooded.resolution));
      assert(gcol >= 0 && grow >= 0 && gcol < flooded.width && grow < flooded.height);
      assert(flooded.cells[static_cast<std::size_t>(grow * flooded.width + gcol)] ==
             0);
    }
  }

  // Production open-room config: viewpoints must stay off the wall rim.
  // Mapping observes frontiers from mid-room; min standoff 0.45 m.
  {
    navigation2d::FrontierExplorerConfig open_config;
    open_config.minimum_frontier_cells = 6;
    open_config.footprint_clearance = .38;
    open_config.minimum_standoff = .45;
    open_config.maximum_standoff = 1.20;
    open_config.required_frontier_observations = 1;
    navigation2d::ExplorationGrid open_room;
    open_room.width = 80;
    open_room.height = 60;
    open_room.resolution = .05;
    open_room.cells.assign(
        static_cast<std::size_t>(open_room.width * open_room.height), 100);
    for (int row = 8; row < 52; ++row)
      for (int col = 8; col < 72; ++col)
        open_room.cells[static_cast<std::size_t>(row * open_room.width + col)] = 0;
    // Unknown strip along the far wall — open-boundary frontier.
    for (int row = 8; row < 52; ++row)
      for (int col = 72; col < 78; ++col)
        open_room.cells[static_cast<std::size_t>(row * open_room.width + col)] = -1;
    navigation2d::FrontierExplorer open_explorer(open_config);
    open_explorer.UpdateMap(open_room);
    const double ox = (20 + .5) * open_room.resolution;
    const double oy = (30 + .5) * open_room.resolution;
    const auto open_goals = open_explorer.SelectGoals(ox, oy);
    assert(!open_goals.empty());
    for (const auto& goal : open_goals) {
      const double standoff =
          std::hypot(goal.x - goal.frontier_x, goal.y - goal.frontier_y);
      assert(standoff + 1e-9 >= open_config.minimum_standoff);
      double nearest_occupied = 10.;
      for (int row = 0; row < open_room.height; ++row) {
        for (int col = 0; col < open_room.width; ++col) {
          if (open_room.cells[static_cast<std::size_t>(row * open_room.width + col)] <=
              0)
            continue;
          const double x = (col + .5) * open_room.resolution;
          const double y = (row + .5) * open_room.resolution;
          nearest_occupied =
              std::min(nearest_occupied, std::hypot(goal.x - x, goal.y - y));
        }
      }
      assert(nearest_occupied + 1e-9 >= open_config.footprint_clearance);
    }
  }

  // Entry-style unknown paint must not turn a long structural bar into a
  // PathCuts-legal chord (UI GLOBAL 穿墙). Paint free past the wall, then
  // PathCuts on the painted grid must still reject.
  {
    navigation2d::ExplorationGrid painted_wall;
    painted_wall.width = 40;
    painted_wall.height = 40;
    painted_wall.resolution = .05;
    painted_wall.origin_x = 0.;
    painted_wall.origin_y = 0.;
    painted_wall.cells.assign(
        static_cast<std::size_t>(painted_wall.width * painted_wall.height), 100);
    const auto pp = [&](int col0, int col1, int row0, int row1,
                        std::int8_t value) {
      for (int row = row0; row < row1; ++row)
        for (int col = col0; col < col1; ++col)
          painted_wall
              .cells[static_cast<std::size_t>(row * painted_wall.width + col)] =
              value;
    };
    pp(4, 36, 4, 20, 0);      // hall
    pp(4, 36, 20, 21, 100);   // full-width structural bar
    pp(4, 36, 21, 36, -1);    // unknown room
    // Simulate the old PathCuts entry paint: unknown → free in a strip.
    const int mid = 20;
    for (int row = 21; row < 30; ++row)
      for (int col = mid - 2; col <= mid + 2; ++col)
        painted_wall
            .cells[static_cast<std::size_t>(row * painted_wall.width + col)] = 0;
    const double rx = (mid + .5) * painted_wall.resolution;
    const double ry = (18 + .5) * painted_wall.resolution;
    const std::vector<std::pair<double, double>> through = {
        {rx, ry},
        {rx, (24 + .5) * painted_wall.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(painted_wall, through, rx, ry));
  }

  // PathCutsLiveOccupied: entry must reject through-wall chords even when the
  // robot stands on the threshold (old 0.35 m under-robot disk hid them).
  {
    navigation2d::ExplorationGrid doorway;
    doorway.width = 40;
    doorway.height = 40;
    doorway.resolution = .05;
    doorway.origin_x = 0.;
    doorway.origin_y = 0.;
    doorway.cells.assign(
        static_cast<std::size_t>(doorway.width * doorway.height), 100);
    const auto dpaint = [&](int col0, int col1, int row0, int row1,
                            std::int8_t value) {
      for (int row = row0; row < row1; ++row)
        for (int col = col0; col < col1; ++col)
          doorway.cells[static_cast<std::size_t>(row * doorway.width + col)] =
              value;
    };
    // South hall free; north pocket free; full-width structural wall at row 20.
    dpaint(4, 36, 4, 20, 0);
    dpaint(4, 36, 21, 36, 0);
    dpaint(4, 36, 20, 21, 100);  // full-width structural bar

    const double robot_x = (20 + .5) * doorway.resolution;  // 1.025
    const double robot_y = (18 + .5) * doorway.resolution;  // 0.925
    // Short GOAL–CARROT chord straight through the wall (~0.30 m), entirely
    // inside the old 0.35 m exemption disk around the robot.
    const std::vector<std::pair<double, double>> through_wall = {
        {robot_x, robot_y},
        {robot_x, (22 + .5) * doorway.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(doorway, through_wall, robot_x,
                                              robot_y));
    // Same chord with the old loose under-robot radius would miss the wall —
    // prove the default 0.22 m still catches it, and that 0.35 m would not
    // be an acceptable production default.
    assert(navigation2d::PathCutsLiveOccupied(
        doorway, through_wall, robot_x, robot_y,
        navigation2d::kPlanningRadius, navigation2d::kPlanningRadius));
    assert(!navigation2d::PathCutsLiveOccupied(
        doorway, through_wall, robot_x, robot_y, navigation2d::kPlanningRadius,
        .40));

    // Path that goes around the wall stays legal.
    const std::vector<std::pair<double, double>> around = {
        {robot_x, robot_y},
        {(8 + .5) * doorway.resolution, robot_y},
        {(8 + .5) * doorway.resolution, (24 + .5) * doorway.resolution},
        {robot_x, (24 + .5) * doorway.resolution},
    };
    assert(!navigation2d::PathCutsLiveOccupied(doorway, around, robot_x,
                                               robot_y));
  }

  // Structural free↔free wall with unknown past a short vestibule: PathCuts
  // must reject through-wall chords (no seal exemption).
  {
    navigation2d::ExplorationGrid alcove_wall;
    alcove_wall.width = 60;
    alcove_wall.height = 40;
    alcove_wall.resolution = .05;
    alcove_wall.origin_x = 0.;
    alcove_wall.origin_y = 0.;
    alcove_wall.cells.assign(
        static_cast<std::size_t>(alcove_wall.width * alcove_wall.height), 100);
    const auto apaint = [&](int col0, int col1, int row0, int row1,
                            std::int8_t value) {
      for (int row = row0; row < row1; ++row)
        for (int col = col0; col < col1; ++col)
          alcove_wall.cells[static_cast<std::size_t>(row * alcove_wall.width +
                                                     col)] = value;
    };
    apaint(2, 58, 4, 18, 0);     // south free
    apaint(2, 58, 18, 19, 100);  // continuous structural wall
    apaint(2, 58, 19, 22, 0);    // short free vestibule north of wall
    apaint(2, 58, 22, 36, -1);   // unfinished unknown within 4 cells of wall
    const double rx = (30 + .5) * alcove_wall.resolution;
    const double ry = (16 + .5) * alcove_wall.resolution;
    const std::vector<std::pair<double, double>> through = {
        {rx, ry},
        {rx, (24 + .5) * alcove_wall.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(alcove_wall, through, rx, ry));
  }

  // Continuous free↔unknown exterior wall: PathCuts rejects through-wall path.
  {
    navigation2d::ExplorationGrid exterior;
    exterior.width = 60;
    exterior.height = 40;
    exterior.resolution = .05;
    exterior.origin_x = 0.;
    exterior.origin_y = 0.;
    exterior.cells.assign(
        static_cast<std::size_t>(exterior.width * exterior.height), 100);
    const auto epaint = [&](int col0, int col1, int row0, int row1,
                            std::int8_t value) {
      for (int row = row0; row < row1; ++row)
        for (int col = col0; col < col1; ++col)
          exterior.cells[static_cast<std::size_t>(row * exterior.width + col)] =
              value;
    };
    epaint(2, 58, 4, 18, 0);     // explored hall
    epaint(2, 58, 18, 19, 100);  // long structural wall (no door gap)
    epaint(2, 58, 19, 36, -1);   // unfinished room unknown
    const double rx = (30 + .5) * exterior.resolution;
    const double ry = (16 + .5) * exterior.resolution;
    const std::vector<std::pair<double, double>> through = {
        {rx, ry},
        {rx, (24 + .5) * exterior.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(exterior, through, rx, ry));
    assert(exterior.cells[18 * exterior.width + 30] == 100);
  }

  // Narrow free kitchen throat (0.40 m): without jamb carve the 0.22 m body
  // clips the posts, so PathCuts rejects. Widen to 0.70 m and the same chord
  // is legal.
  {
    navigation2d::ExplorationGrid pinch;
    pinch.width = 60;
    pinch.height = 40;
    pinch.resolution = .05;
    pinch.origin_x = 0.;
    pinch.origin_y = 0.;
    pinch.cells.assign(static_cast<std::size_t>(pinch.width * pinch.height), 100);
    const auto pp = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row)
        for (int col = col0; col < col1; ++col)
          pinch.cells[static_cast<std::size_t>(row * pinch.width + col)] = value;
    };
    pp(2, 58, 4, 18, 0);       // hall
    pp(2, 58, 22, 36, -1);     // kitchen unknown
    pp(24, 32, 18, 22, 0);     // 0.40 m free throat (8 cells)
    const double rx = (28 + .5) * pinch.resolution;
    const double ry = (17 + .5) * pinch.resolution;
    const std::vector<std::pair<double, double>> entry = {
        {rx, ry},
        {rx, (24 + .5) * pinch.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(pinch, entry, rx, ry));
    pp(20, 34, 18, 22, 0);  // widen to 0.70 m
    assert(!navigation2d::PathCutsLiveOccupied(pinch, entry, rx, ry));
  }

  // Free↔free pinch (0.25 m): without jamb carve the 0.22 m body clips the
  // posts, so PathCuts rejects the entry chord.
  {
    navigation2d::ExplorationGrid both_free;
    both_free.width = 60;
    both_free.height = 40;
    both_free.resolution = .05;
    both_free.origin_x = 0.;
    both_free.origin_y = 0.;
    both_free.cells.assign(
        static_cast<std::size_t>(both_free.width * both_free.height), 100);
    const auto bp = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row)
        for (int col = col0; col < col1; ++col)
          both_free.cells[static_cast<std::size_t>(row * both_free.width + col)] =
              value;
    };
    bp(2, 58, 4, 18, 0);      // hall (known free)
    bp(2, 58, 22, 36, 0);     // room (known free)
    bp(27, 32, 18, 22, 0);    // 0.25 m free throat (5 cells)
    const double rx = (29 + .5) * both_free.resolution;
    const double ry = (17 + .5) * both_free.resolution;
    const std::vector<std::pair<double, double>> entry = {
        {rx, ry},
        {rx, (24 + .5) * both_free.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(both_free, entry, rx, ry));
  }

  // Occupied door-width kitchen bar: PathCuts rejects until SLAM clears it.
  {
    navigation2d::ExplorationGrid sealed;
    sealed.width = 60;
    sealed.height = 40;
    sealed.resolution = .05;
    sealed.origin_x = 0.;
    sealed.origin_y = 0.;
    sealed.cells.assign(static_cast<std::size_t>(sealed.width * sealed.height),
                        100);
    const auto spaint = [&](int col0, int col1, int row0, int row1,
                            std::int8_t value) {
      for (int row = row0; row < row1; ++row)
        for (int col = col0; col < col1; ++col)
          sealed.cells[static_cast<std::size_t>(row * sealed.width + col)] =
              value;
    };
    spaint(2, 58, 4, 18, 0);      // hall
    spaint(2, 58, 22, 36, -1);    // kitchen unknown
    spaint(20, 34, 18, 22, 0);    // 0.70 m mouth vestibule
    spaint(20, 34, 20, 21, 100);  // thin occupied bar
    const double rx = (27 + .5) * sealed.resolution;
    const double ry = (17 + .5) * sealed.resolution;
    const std::vector<std::pair<double, double>> entry = {
        {rx, ry},
        {rx, (24 + .5) * sealed.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(sealed, entry, rx, ry));
  }

  // Free↔free SLAM seal 0.30 m thick: PathCuts rejects the through-seal chord.
  {
    navigation2d::ExplorationGrid thick_seal;
    thick_seal.width = 60;
    thick_seal.height = 40;
    thick_seal.resolution = .05;
    thick_seal.origin_x = 0.;
    thick_seal.origin_y = 0.;
    thick_seal.cells.assign(
        static_cast<std::size_t>(thick_seal.width * thick_seal.height), 100);
    const auto tp = [&](int col0, int col1, int row0, int row1,
                        std::int8_t value) {
      for (int row = row0; row < row1; ++row)
        for (int col = col0; col < col1; ++col)
          thick_seal.cells[static_cast<std::size_t>(row * thick_seal.width +
                                                    col)] = value;
    };
    tp(2, 58, 4, 18, 0);       // hall
    tp(2, 58, 24, 36, 0);      // room (known free)
    tp(22, 34, 18, 24, 100);   // 0.30 m × 0.60 m occupied door seal
    const double rx = (28 + .5) * thick_seal.resolution;
    const double ry = (17 + .5) * thick_seal.resolution;
    const std::vector<std::pair<double, double>> entry = {
        {rx, ry},
        {rx, (26 + .5) * thick_seal.resolution},
    };
    assert(navigation2d::PathCutsLiveOccupied(thick_seal, entry, rx, ry));
  }

  // Wall-tip thrash: abandon only after repeated live-occupied cuts. A single
  // low-clearance graze near a passable door must replan, not leave the room.
  assert(!navigation2d::PreferTipAbandonAfterPathCut(0, .30));
  assert(!navigation2d::PreferTipAbandonAfterPathCut(1, .30));
  assert(navigation2d::PreferTipAbandonAfterPathCut(2, .30));
  assert(!navigation2d::PreferTipAbandonAfterPathCut(1, .08));  // tip graze → replan
  assert(!navigation2d::PreferTipAbandonAfterPathCut(0, .10));
  assert(!navigation2d::PreferTipAbandonAfterPathCut(0, 0.));

  // Diagonal corner cut: centerline misses the occupied cell but the body
  // clips the wall corner. Must still count as 穿墙 (entry body ≥0.14).
  {
    navigation2d::ExplorationGrid corner;
    corner.width = 40;
    corner.height = 40;
    corner.resolution = .05;
    corner.origin_x = 0.;
    corner.origin_y = 0.;
    corner.cells.assign(static_cast<std::size_t>(corner.width * corner.height), 0);
    // Solid L / block in the middle; path from SW free to NE free clips corner.
    for (int row = 16; row < 28; ++row)
      for (int col = 16; col < 28; ++col)
        corner.cells[static_cast<std::size_t>(row * corner.width + col)] = 100;
    const double x0 = (12 + .5) * corner.resolution;
    const double y0 = (12 + .5) * corner.resolution;
    const double x1 = (30 + .5) * corner.resolution;
    const double y1 = (30 + .5) * corner.resolution;
    const std::vector<std::pair<double, double>> diag = {{x0, y0}, {x1, y1}};
    assert(navigation2d::PathCutsLiveOccupied(corner, diag, x0, y0, .14));
    assert(navigation2d::PathCutsLiveOccupied(corner, diag, x0, y0, .10));
  }

  // Corridor island: chord through a solid pillar must reject.
  {
    navigation2d::ExplorationGrid island;
    island.width = 60;
    island.height = 40;
    island.resolution = .05;
    island.origin_x = 0.;
    island.origin_y = 0.;
    island.cells.assign(static_cast<std::size_t>(island.width * island.height), 100);
    const auto ip = [&](int col0, int col1, int row0, int row1, std::int8_t value) {
      for (int row = row0; row < row1; ++row)
        for (int col = col0; col < col1; ++col)
          island.cells[static_cast<std::size_t>(row * island.width + col)] = value;
    };
    ip(2, 58, 8, 32, 0);       // open hall
    ip(28, 34, 16, 24, 100);   // 0.30×0.40 m solid island
    const double x0 = (20 + .5) * island.resolution;
    const double y0 = (20 + .5) * island.resolution;
    const double x1 = (40 + .5) * island.resolution;
    const double y1 = (20 + .5) * island.resolution;
    const std::vector<std::pair<double, double>> through = {{x0, y0}, {x1, y1}};
    assert(navigation2d::PathCutsLiveOccupied(
        island, through, x0, y0, navigation2d::kPlanningRadius));
    assert(island.cells[20 * island.width + 30] == 100);
  }

  // Open-wall frontier expansion is not a corridor mouth even when the free
  // span reads large (live gap=1.65 m along a wall → false "easy door").
  {
    navigation2d::ExplorationGrid open_wall;
    open_wall.width = 60;
    open_wall.height = 40;
    open_wall.resolution = .05;
    open_wall.cells.assign(
        static_cast<std::size_t>(open_wall.width * open_wall.height), 100);
    for (int row = 4; row < 36; ++row)
      for (int col = 4; col < 56; ++col)
        open_wall.cells[static_cast<std::size_t>(row * open_wall.width + col)] =
            0;
    // North unknown edge — open expansion, not a door throat.
    for (int col = 4; col < 56; ++col)
      open_wall.cells[static_cast<std::size_t>(35 * open_wall.width + col)] = -1;
    navigation2d::FrontierExplorerConfig ocfg;
    ocfg.footprint_clearance = navigation2d::kPlanningRadius;
    ocfg.minimum_frontier_cells = 4;
    ocfg.minimum_standoff = .10;
    ocfg.maximum_standoff = 1.10;
    ocfg.required_frontier_observations = 1;
    navigation2d::FrontierExplorer oex(ocfg);
    oex.UpdateMap(open_wall);
    const double fx = (30 + .5) * open_wall.resolution;
    const double fy = (34 + .5) * open_wall.resolution;
    assert(!oex.IsCorridorMouth(fx, fy));
    assert(oex.MouthGapWidth(fx, fy) > 1.40);
  }

  // Wide free gap into a deep unknown room: both free-run axes hit the probe
  // cap (same geometry as open-wall fringe), but unknown depth ≥0.55 m must
  // still count as a corridor mouth so unfinished rooms stay enterable.
  {
    navigation2d::ExplorationGrid wide_room;
    wide_room.width = 50;
    wide_room.height = 50;
    wide_room.resolution = .05;
    wide_room.cells.assign(
        static_cast<std::size_t>(wide_room.width * wide_room.height), 100);
    for (int row = 4; row < 28; ++row)
      for (int col = 4; col < 46; ++col)
        wide_room.cells[static_cast<std::size_t>(row * wide_room.width + col)] =
            0;
    // Deep unknown room north of a wide free edge.
    for (int row = 28; row < 46; ++row)
      for (int col = 10; col < 40; ++col)
        wide_room.cells[static_cast<std::size_t>(row * wide_room.width + col)] =
            -1;
    navigation2d::FrontierExplorerConfig wcfg;
    wcfg.footprint_clearance = navigation2d::kPlanningRadius;
    wcfg.minimum_frontier_cells = 4;
    wcfg.required_frontier_observations = 1;
    navigation2d::FrontierExplorer wex(wcfg);
    wex.UpdateMap(wide_room);
    const double wfx = (25 + .5) * wide_room.resolution;
    const double wfy = (27 + .5) * wide_room.resolution;
    assert(wex.IsCorridorMouth(wfx, wfy));
  }

  // Easy shallow doorway: free gap ≥0.50 m but room only ~0.40 m deep.
  // Fixed 0.55 m entry minimum used to return nullopt and abandon the mouth
  // while the chassis sat on a clearly passable threshold (UI red box).
  {
    navigation2d::ExplorationGrid shallow;
    shallow.width = 50;
    shallow.height = 40;
    shallow.resolution = .05;
    shallow.origin_x = 0.;
    shallow.origin_y = 0.;
    shallow.cells.assign(static_cast<std::size_t>(shallow.width * shallow.height),
                         100);
    const auto sp = [&](int col0, int col1, int row0, int row1,
                        std::int8_t value) {
      for (int row = row0; row < row1; ++row)
        for (int col = col0; col < col1; ++col)
          shallow.cells[static_cast<std::size_t>(row * shallow.width + col)] =
              value;
    };
    sp(2, 48, 4, 22, 0);        // hall
    sp(18, 32, 22, 30, 0);      // 0.70 m × 0.40 m shallow room (free)
    sp(18, 32, 30, 36, -1);     // unknown past the pocket
    navigation2d::FrontierExplorerConfig scfg;
    scfg.footprint_clearance = navigation2d::kPlanningRadius;
    scfg.minimum_frontier_cells = 4;
    scfg.minimum_standoff = .10;
    scfg.maximum_standoff = 1.10;
    scfg.required_frontier_observations = 1;
    navigation2d::FrontierExplorer sexplorer(scfg);
    sexplorer.UpdateMap(shallow);
    navigation2d::ExplorationGoal mouth;
    mouth.x = (25 + .5) * shallow.resolution;
    mouth.y = (20 + .5) * shallow.resolution;
    mouth.frontier_x = (25 + .5) * shallow.resolution;
    mouth.frontier_y = (22 + .5) * shallow.resolution;
    mouth.yaw = 1.57;
    const double gap =
        sexplorer.MouthGapWidth(mouth.frontier_x, mouth.frontier_y);
    assert(gap + 1e-9 >= .50);
    const double free_depth =
        sexplorer.MouthFreeDepth(mouth, mouth.x, mouth.y - .3);
    assert(free_depth + 1e-9 >= .30);
    // Deep schedule (old StartEntryProbe) must fail; shallow schedule works.
    assert(!sexplorer.SelectEntryProbe(mouth, mouth.x, mouth.y - .3, .55, 1.30));
    const auto poke =
        sexplorer.SelectEntryProbe(mouth, mouth.x, mouth.y - .3, .22, .55);
    assert(poke.has_value());
    assert(poke->y > mouth.frontier_y - 1e-6);
    // Mid-gap: not glued to either jamb of the 0.70 m mouth.
    assert(poke->x > (18 + .5) * shallow.resolution + .14);
    assert(poke->x < (32 + .5) * shallow.resolution - .14);
  }

  // Alcove partition finger: thin occupied fin must stay lethal to PathCuts.
  {
    navigation2d::ExplorationGrid finger;
    finger.width = 60;
    finger.height = 48;
    finger.resolution = .05;
    finger.origin_x = 0.;
    finger.origin_y = 0.;
    finger.cells.assign(static_cast<std::size_t>(finger.width * finger.height),
                        100);
    const auto fp = [&](int col0, int col1, int row0, int row1,
                        std::int8_t value) {
      for (int row = row0; row < row1; ++row)
        for (int col = col0; col < col1; ++col)
          finger.cells[static_cast<std::size_t>(row * finger.width + col)] =
              value;
    };
    fp(2, 58, 4, 36, 0);        // open room
    fp(2, 58, 36, 44, 100);     // north wall mass
    fp(28, 31, 20, 36, 100);    // 0.15×0.80 m fin hanging south from wall
    const double x0 = (20 + .5) * finger.resolution;
    const double y0 = (28 + .5) * finger.resolution;
    const double x1 = (38 + .5) * finger.resolution;
    const double y1 = (28 + .5) * finger.resolution;
    const std::vector<std::pair<double, double>> through_fin = {{x0, y0},
                                                                {x1, y1}};
    assert(navigation2d::PathCutsLiveOccupied(
        finger, through_fin, x0, y0, navigation2d::kPlanningRadius));
    assert(finger.cells[28 * finger.width + 29] == 100);
    assert(finger.cells[21 * finger.width + 29] == 100);
  }

  // Doorway peek: the frontier steps farther than the 0.35 m association
  // disk, but a room of unknown is still beyond the mouth. That viewpoint
  // is not done.
  {
    navigation2d::FrontierExplorerConfig room_config;
    room_config.minimum_frontier_cells = 4;
    room_config.footprint_clearance = navigation2d::kPlanningRadius;
    room_config.required_frontier_observations = 1;
    navigation2d::ExplorationGrid room;
    room.width = 80;
    room.height = 70;
    room.resolution = .05;
    room.cells.assign(static_cast<std::size_t>(room.width * room.height), 100);
    const auto paint = [&](int col0, int col1, int row0, int row1,
                           std::int8_t value) {
      for (int row = row0; row < row1; ++row)
        for (int col = col0; col < col1; ++col)
          room.cells[static_cast<std::size_t>(row * room.width + col)] = value;
    };
    paint(8, 70, 8, 30, 0);     // hall
    paint(24, 56, 30, 40, 0);   // 1.6 m throat, peeked 0.50 m into the room
    paint(16, 64, 40, 62, -1);  // room still unknown
    navigation2d::FrontierExplorer rooms(room_config);
    rooms.UpdateMap(room);
    navigation2d::ExplorationGoal mouth;
    mouth.x = (40 + .5) * room.resolution;
    mouth.y = (20 + .5) * room.resolution;
    mouth.frontier_x = (40 + .5) * room.resolution;
    mouth.frontier_y = (30 + .5) * room.resolution;
    assert(!rooms.GoalRegionStillFrontier(mouth));
    assert(rooms.UnenteredRoomBeyond(mouth));
    paint(16, 64, 40, 62, 0);  // room fully observed
    rooms.UpdateMap(room);
    assert(!rooms.UnenteredRoomBeyond(mouth));
  }

  {
    // A 0.72 m kitchen fully seen through its door from the hallway has no
    // frontier, yet the robot never drove in. It must still be a visit goal.
    navigation2d::FrontierExplorer visits(config);
    navigation2d::ExplorationGrid flat;
    flat.width = 140;
    flat.height = 130;
    flat.resolution = .03;
    flat.cells.assign(static_cast<std::size_t>(flat.width) * flat.height, 100);
    for (int row = 5; row < 39; ++row)
      for (int col = 5; col < 135; ++col) flat.cells[row * flat.width + col] = 0;
    for (int row = 39; row < 126; ++row)
      for (int col = 60; col < 84; ++col) flat.cells[row * flat.width + col] = 0;
    visits.UpdateMap(flat);
    std::vector<std::pair<double, double>> trail;
    for (double x = .3; x < 3.9; x += .05) trail.emplace_back(x, .66);
    const auto unvisited = visits.UnvisitedSpaceGoals(
        trail, navigation2d::kOpenRetreat,
        navigation2d::kRoomPocketDepthM * navigation2d::kRoomPocketDepthM);
    assert(unvisited.size() == 1);
    assert(unvisited.front().y > 1.2);
    assert(unvisited.front().x > 1.8 && unvisited.front().x < 2.52);
    for (double y = .66; y < 3.6; y += .05) trail.emplace_back(2.16, y);
    assert(visits.UnvisitedSpaceGoals(
        trail, navigation2d::kOpenRetreat,
        navigation2d::kRoomPocketDepthM * navigation2d::kRoomPocketDepthM).empty());
  }

  {
    // Viewpoint 0.21 m past a door-post end: one more scanned cell makes it
    // "goal occupied". Settling moves it clear of the post by one cell.
    navigation2d::FrontierExplorer settle(config);
    navigation2d::ExplorationGrid open;
    open.width = open.height = 100;
    open.resolution = .03;
    open.cells.assign(static_cast<std::size_t>(open.width) * open.height, 0);
    for (int i = 0; i < 100; ++i) {
      open.cells[i] = open.cells[99 * 100 + i] = 100;
      open.cells[i * 100] = open.cells[i * 100 + 99] = 100;
    }
    for (int row = 30; row < 50; ++row)
      for (int col = 48; col < 52; ++col) open.cells[row * 100 + col] = 100;
    settle.UpdateMap(open);
    navigation2d::ExplorationGoal tight;
    tight.x = 1.50;
    tight.y = 1.50 + .21;
    tight.frontier_x = 1.5;
    tight.frontier_y = 2.4;
    const auto moved = settle.SettleGoal(tight);
    assert(moved);
    assert(moved->y > tight.y);
    assert(std::hypot(moved->x - tight.x, moved->y - tight.y) <= navigation2d::kSwingClearance);
    assert(moved->frontier_x == tight.frontier_x && moved->frontier_y == tight.frontier_y);
    navigation2d::ExplorationGoal roomy = tight;
    roomy.y = 2.40;
    const auto same = settle.SettleGoal(roomy);
    assert(same && same->x == roomy.x && same->y == roomy.y);
  }

}
