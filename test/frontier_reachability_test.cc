#include "navigation2d/exploration/frontier_reachability.h"

#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

#include "navigation2d/exploration/viewpoint_settlement.h"
int main() {
  using namespace navigation2d;
  using namespace navigation2d::frontier_internal;
  ExplorationGrid map{40, 30, .1, 0., 0., std::vector<std::int8_t>(1200, 0)};
  for (int row = 0; row < 30; ++row) {
    map.cells[row * 40 + 20] = 100;
  }
  auto observation = map;
  FrontierReachability reachability(map, observation, .38);
  assert(!reachability.SameRegion(.65, 1.55, 3.45, 1.55, 10.));
  assert(reachability.SameRegion(.65, 1.55, 1.45, 1.55, .79));
  assert(!reachability.SameRegion(.65, 1.55, 1.45, 1.55, .7));
  assert(reachability.SameRegion(.65, 1.55, .65, 1.55, 0.));
  assert(!reachability.SameRegion(.65, 1.55, .65, 1.55, -1.));
  assert(reachability.UnknownCells(.65, 1.55) == 0);
  for (int row = 10; row < 15; ++row) {
    for (int col = 5; col < 10; ++col) {
      observation.cells[row * 40 + col] = -1;
    }
  }
  for (int row = 10; row < 15; ++row) {
    for (int col = 28; col < 36; ++col) {
      observation.cells[row * 40 + col] = -1;
    }
  }
  // Count carved original unknown only in the connected planning component.
  assert(reachability.UnknownCells(.65, 1.55) == 25);
  ExplorationGoal goal;
  goal.x = 2.05;
  goal.y = 1.55;
  goal.yaw = .7;
  goal.frontier_x = 2.5;
  goal.region_id = 42;
  goal.score = 7.;
  goal.leftover_approach = true;
  const auto moved = SettleViewpoint(map, observation, .38, goal);
  assert(moved && std::abs(moved->x - 1.65) < 1e-12 && std::abs(moved->y - 1.55) < 1e-12);
  assert(moved->yaw == .7 && moved->frontier_x == 2.5 && moved->region_id == 42);
  assert(moved->score == 7. && moved->leftover_approach);
  const auto unchanged = SettleViewpoint(map, observation, .38, *moved);
  assert(unchanged && unchanged->x == moved->x && unchanged->y == moved->y);
  goal.x = std::numeric_limits<double>::quiet_NaN();
  assert(!SettleViewpoint(map, observation, .38, goal));
  assert(!SettleViewpoint({}, {}, .38, goal));
  const ExplorationGrid empty;
  const FrontierReachability unavailable(empty, empty, .38);
  assert(!unavailable.SameRegion(0., 0., 1., 1., 1.) && unavailable.UnknownCells(0., 0.) == 0);
}
