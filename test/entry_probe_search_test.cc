#include "navigation2d/exploration/entry_probe_search.h"

#include <cmath>
#include <source_location>
#include <stdexcept>
#include <string>
namespace {
void Require(bool condition, std::source_location location = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("entry probe contract at " + std::to_string(location.line()));
  }
}
}  // namespace
int main() {
  using namespace navigation2d;
  using namespace navigation2d::frontier_internal;
  ExplorationGrid map{20, 20, .1, 0., 0., std::vector<std::int8_t>(400, 0)};
  ExplorationGoal goal;
  goal.region_id = 73;
  goal.frontier_cells = 17;
  goal.leftover_approach = true;
  goal.x = .5;
  goal.y = 1.;
  goal.frontier_x = 1.;
  goal.frontier_y = 1.;
  auto direction = EntryProbeDirection(map, map, goal, .5, 1.);
  Require(direction && direction->first == 1. && direction->second == 0.);
  direction = EntryProbeDirection(map, map, goal, 1., 1.);
  Require(direction && direction->first == 1. && direction->second == 0.);
  auto ambiguous = goal;
  ambiguous.x = ambiguous.frontier_x;
  Require(!EntryProbeDirection(map, map, ambiguous, 1., 1.));
  auto observed = map;
  // The observed unknown centroid takes precedence over the approach side.
  observed.cells[14 * 20 + 9] = -1;
  observed.cells[14 * 20 + 10] = -1;
  observed.cells[14 * 20 + 11] = -1;
  direction = EntryProbeDirection(map, observed, goal, .5, 1.);
  Require(direction && direction->first > 0. && direction->second > .9);
  EntryProbeGrid grid(map, map, 0.);
  Require(grid.Allows(10, 10) && !grid.Allows(-1, 10));
  const auto distances = grid.Distances(.55, .55);
  Require(distances.size() == 400 && distances[5 * 20 + 5] == 0);
  Require(distances[5 * 20 + 6] == 1 && distances[6 * 20 + 6] == 2);
  Require(MouthGapWidthAt(map, 1., 1.) > 0. && MouthGapWidthAt(map, -1., 1.) == 0.);
  const auto probe = SelectEntryProbeAt(map, map, 0., goal, .5, 1., .22, .70);
  Require(probe && probe->x > goal.frontier_x && probe->frontier_x == goal.frontier_x);
  Require(probe->region_id == goal.region_id && probe->frontier_cells == 17 &&
          probe->leftover_approach && probe->yaw == 0.);
  Require(!SelectEntryProbeAt(map, map, 0., goal, .5, 1., 0., .7));
  Require(!SelectEntryProbeAt(map, map, 0., goal, .5, 1., .7, .2));
  auto blocked = map;
  blocked.cells.assign(400, 100);
  Require(EntryProbeGrid(blocked, blocked, 0.).Distances(.5, 1.).empty());
  Require(!SelectEntryProbeAt(blocked, blocked, 0., goal, .5, 1., .22, .7));
  // Preserve geometric fallback into a separately observed pocket when no
  // candidate belongs to the robot's four-neighbour component.
  auto islands = blocked;
  for (int row = 3; row < 17; ++row) {
    for (int col = 1; col < 7; ++col) {
      islands.cells[row * 20 + col] = 0;
    }
    for (int col = 13; col < 19; ++col) {
      islands.cells[row * 20 + col] = 0;
    }
  }
  const auto fallback = SelectEntryProbeAt(islands, islands, 0., goal, .35, 1., .22, .70);
  Require(fallback && fallback->x > 1.3 && fallback->region_id == 73);
  auto unseen = map;
  unseen.cells.assign(400, -1);
  Require(EntryProbeGrid(map, unseen, 0.).Distances(.55, .55).empty());
}
