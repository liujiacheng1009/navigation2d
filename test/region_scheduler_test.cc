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
  // Merely inspecting another region's candidates must not steal the DFS
  // context. Only successful dispatch is allowed to commit that transition.
  // BLOCKED preferred (raw frontiers, 0 candidates) soft-unlocks a nearby
  // sibling but keeps the active region on the stack.
  scheduler.Enter(again.front());
  const auto offered = scheduler.Order({far}, .8, 1.2);
  assert(!offered.empty());
  assert(offered.front().region_id == right);
  assert(scheduler.ActiveRegion() == left);
  assert(scheduler.RegionAt(.8, 1.2) != 0);
  scheduler.Enter(again.front());
  assert(scheduler.Order({far, near}, .8, 1.2).front().region_id == left);

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
  assert(scheduler.Order({far, near}, .8, 1.2).front().region_id == left);
  // A disappearing speck must reuse its identity, without borrowing a room ID.
  const auto speck = scheduler.RegionAt(3.05, .15);
  const auto count_before = scheduler.regions().size();
  for (int cycle = 0; cycle < 100; ++cycle) {
    auto missing = padded;
    missing.cells[missing.width + 32] = 100;
    scheduler.Update(missing, .20);
    scheduler.Update(padded, .20);
    assert(scheduler.RegionAt(3.05, .15) == speck);
    assert(scheduler.regions().size() == count_before);
  }
  // Merge two disconnected corridors, then split them repeatedly.
  navigation2d::ExplorationGrid corridors;
  corridors.width = 30; corridors.height = 5; corridors.resolution = .1;
  corridors.cells.assign(150, 100);
  for (int col = 2; col < 28; ++col) corridors.cells[60 + col] = 0;
  corridors.cells[75] = 100;
  navigation2d::RegionScheduler split_scheduler;
  split_scheduler.Update(corridors, .2);
  const auto first = split_scheduler.RegionAt(.55, .25);
  const auto second = split_scheduler.RegionAt(2.55, .25);
  assert(first && second && first != second);
  for (int cycle = 0; cycle < 100; ++cycle) {
    auto merged = corridors;
    merged.cells[75] = 0;
    split_scheduler.Update(merged, .2);
    split_scheduler.Update(corridors, .2);
    assert(split_scheduler.RegionAt(.55, .25) == first);
    assert(split_scheduler.RegionAt(2.55, .25) == second);
    assert(split_scheduler.regions().size() == 2);
  }
  // An actually new island gets a distinct ID and remains available.
  corridors.cells[31] = 0;
  split_scheduler.Update(corridors, .2);
  const auto fresh = split_scheduler.RegionAt(.15, .15);
  assert(fresh && fresh != first && fresh != second);
  corridors.cells[31] = 100;
  for (int i = 0; i < 70; ++i) split_scheduler.Update(corridors, .2);
  assert(split_scheduler.regions().count(fresh) == 0);

  // Once a room is entered, finish its observations before leaving — even if
  // a sibling room offers a higher score.
  navigation2d::RegionScheduler finish_first;
  finish_first.Update(grid, .20);
  navigation2d::ExplorationGoal left_low = near;
  left_low.score = 1.;
  left_low.information_gain = 1.;
  navigation2d::ExplorationGoal right_high = far;
  right_high.score = 100.;
  right_high.information_gain = 100.;
  finish_first.Enter(left_low);
  assert(finish_first.ActiveRegion() == left);
  assert(finish_first.InActiveContext(left));
  assert(!finish_first.InActiveContext(right));
  const auto locked = finish_first.Order({right_high, left_low}, .8, 1.2);
  assert(locked.front().region_id == left);
  assert(locked.back().region_id == right);
  // BLOCKED tip (raw frontiers, zero in-room candidates): soft-unlock siblings
  // so large PENDING rooms are not discarded, but keep the preferred region
  // on the DFS stack for later resume.
  const auto soft = finish_first.Order({right_high}, .8, 1.2);
  assert(!soft.empty());
  assert(soft.front().region_id == right);
  assert(finish_first.ActiveRegion() == left);
  assert(finish_first.regions().at(left).frontier_cells > 0);

  // Far unfinished room beyond the old 4 m soft-unlock erase must still be
  // admitted — returning {} here caused PARTIAL while visible alcoves remained.
  navigation2d::ExplorationGoal farther = right_high;
  farther.x = 5.5;
  farther.y = 1.2;
  farther.frontier_x = 5.5;
  farther.frontier_y = 1.2;
  const auto far_only = finish_first.Order({farther}, .8, 1.2);
  assert(!far_only.empty());
  assert(far_only.front().region_id == right);
  assert(finish_first.ActiveRegion() == left);

  // BLOCKED tip with a local mouth must not starve a large unfinished sibling:
  // tip thrash ranks after coverage work elsewhere.
  navigation2d::ExplorationGoal tip_mouth = left_low;
  tip_mouth.score = 50.;
  tip_mouth.information_gain = 50.;
  const auto leave_tip = finish_first.Order({tip_mouth, farther}, .8, 1.2);
  assert(!leave_tip.empty());
  assert(leave_tip.front().region_id == right);
  assert(finish_first.ActiveRegion() == left);
  return 0;
}
