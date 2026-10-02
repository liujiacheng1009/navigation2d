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
  assert(ordered.front().region_id == right);
  assert(ordered.back().region_id == left);

  // Reordering after a map update keeps both regions eligible. The scheduler
  // never drops a neighbouring candidate merely because the active region was
  // selected first.
  scheduler.Enter(ordered.front());
  scheduler.Update(grid, .20);
  const auto again = scheduler.Order({far, near}, .8, 1.2);
  assert(again.size() == 2);
  assert(again.front().region_id == right);
  // Merely inspecting another region's candidates must not steal the DFS
  // context. Only successful dispatch is allowed to commit that transition.
  scheduler.Enter(again.front());
  const auto offered = scheduler.Order({far}, .8, 1.2);
  assert(!offered.empty());
  assert(!scheduler.stack().empty());
  scheduler.Enter(again.front());
  assert(scheduler.Order({far, near}, .8, 1.2).front().region_id == right);

  // Narrow disconnected free components must not disappear when other rooms
  // have wide cores (previously RegionAt returned 0 for these cells).
  grid.cells[1 * grid.width + 30] = 0;
  scheduler.Update(grid, .20);
  assert(scheduler.RegionAt(3.05, .15) != 0);

  // Map padding changes indices, not the identity of an active room.
  auto padded = grid;
  padded.width += 2;
  padded.origin_x = -.2;
  padded.cells.assign(padded.width * padded.height, 100);
  for (int row = 0; row < grid.height; ++row)
    for (int col = 0; col < grid.width; ++col)
      padded.cells[row * padded.width + col + 2] = grid.cells[row * grid.width + col];
  scheduler.Update(padded, .20);
  assert(scheduler.RegionAt(.8, 1.2) == left);
  assert(scheduler.Order({far, near}, .8, 1.2).front().region_id == right);
  return 0;
}
