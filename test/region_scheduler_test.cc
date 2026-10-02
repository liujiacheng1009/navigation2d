#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/exploration/region_scheduler.h"

#include <cassert>
#include <cstdint>
#include <vector>

int main() {
  navigation2d::ExplorationGrid grid;
  grid.width = 60;
  grid.height = 24;
  grid.resolution = .10;
  grid.cells.assign(static_cast<std::size_t>(grid.width * grid.height), 100);
  // Two rooms are separated by a wall. Their unknown-facing free cells are
  // distinct regions, so a goal in the robot's room must be selected first.
  for (int row = 3; row < 21; ++row) {
    for (int col = 3; col < 22; ++col)
      grid.cells[static_cast<std::size_t>(row * grid.width + col)] = 0;
    for (int col = 38; col < 57; ++col)
      grid.cells[static_cast<std::size_t>(row * grid.width + col)] = 0;
  }
  for (int row = 8; row < 16; ++row)
    for (int col = 22; col < 38; ++col)
      grid.cells[static_cast<std::size_t>(row * grid.width + col)] = -1;

  navigation2d::RegionScheduler scheduler;
  scheduler.Update(grid, .20);
  const auto left = scheduler.RegionAt(.8, 1.2);
  const auto right = scheduler.RegionAt(4.8, 1.2);
  assert(left != 0 && right != 0 && left != right);

  navigation2d::ExplorationGoal far;
  far.x = 4.8; far.y = 1.2; far.frontier_x = 4.8; far.frontier_y = 1.2;
  far.score = 100.;
  navigation2d::ExplorationGoal near;
  near.x = .8; near.y = 1.2; near.frontier_x = 1.8; near.frontier_y = 1.2;
  near.score = 1.;
  const auto ordered = scheduler.Order({far, near}, .8, 1.2);
  assert(ordered.size() == 2);
  assert(ordered.front().region_id == left);
  assert(ordered.back().region_id == right);

  // Reordering after a map update keeps both regions eligible. The scheduler
  // never drops a neighbouring candidate merely because the active region was
  // selected first.
  scheduler.Enter(ordered.front());
  scheduler.Update(grid, .20);
  const auto again = scheduler.Order({far, near}, .8, 1.2);
  assert(again.size() == 2);
  assert(again.front().region_id == left);
  return 0;
}
