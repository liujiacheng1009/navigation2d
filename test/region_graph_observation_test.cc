#include <cassert>
#include <iostream>
#include <string_view>

#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/exploration/region_scheduler.h"
using namespace navigation2d;
namespace {
void Snapshot(const RegionScheduler& scheduler, std::string_view stage) {
  std::cout << stage << ':';
  for (const auto& [id, r] : scheduler.regions()) {
    std::cout << id << ',' << r.parent << ',' << static_cast<int>(r.state) << ',' << r.visible
              << ',' << r.passage << ',' << r.free_cells << ',' << r.frontier_cells << ','
              << r.candidates << ',' << r.stable_empty_updates << ',' << r.evidence << '[';
    for (const auto neighbor : r.neighbors) {
      std::cout << neighbor << ',';
    }
    std::cout << "];";
  }
  std::cout << '\n';
}
}  // namespace
int main() {
  ExplorationGrid grid{20, 12, .1, 0., 0., std::vector<std::int8_t>(240, 100)};
  for (int y = 2; y < 10; ++y) {
    for (int x = 2; x < 8; ++x) {
      grid.cells[y * grid.width + x] = 0;
    }
    for (int x = 12; x < 18; ++x) {
      grid.cells[y * grid.width + x] = 0;
    }
  }
  for (int y = 5; y < 7; ++y) {
    for (int x = 8; x < 12; ++x) {
      grid.cells[y * grid.width + x] = -1;
    }
  }
  RegionScheduler scheduler;
  scheduler.Update(grid, .15);
  const auto left = scheduler.RegionAt(.45, .65), right = scheduler.RegionAt(1.45, .65);
  assert(left && right && left != right);
  assert(scheduler.regions().at(left).frontier_cells > 0);
  assert(scheduler.regions().at(right).frontier_cells > 0);
  const auto evidence = scheduler.regions().at(left).evidence;
  Snapshot(scheduler, "frontier");
  scheduler.Update(grid, .15);
  assert(scheduler.regions().at(left).evidence == evidence);
  Snapshot(scheduler, "unchanged");
  // Clear transient passage/candidate facts on the next complete observation.
  scheduler.MarkPassage(left);
  assert(scheduler.HasPassage(left));
  auto padded = grid;
  padded.width += 2;
  padded.origin_x = -.2;
  padded.cells.assign(padded.width * padded.height, 100);
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      padded.cells[y * padded.width + x + 2] = grid.cells[y * grid.width + x];
    }
  }
  scheduler.Update(padded, .15);
  assert(scheduler.RegionAt(.45, .65) == left && !scheduler.HasPassage(left));
  assert(scheduler.regions().at(left).evidence == evidence);
  Snapshot(scheduler, "padded");
  for (auto& cell : padded.cells) {
    if (cell < 0) {
      cell = 100;
    }
  }
  scheduler.Update(padded, .15);
  assert(scheduler.regions().at(left).stable_empty_updates == 1);
  Snapshot(scheduler, "empty-first");
  scheduler.Update(padded, .15);
  assert(scheduler.regions().at(left).state == RegionState::kComplete);
  Snapshot(scheduler, "empty-stable");
  scheduler.Update(grid, .15);
  assert(scheduler.regions().at(left).state == RegionState::kPending);
  Snapshot(scheduler, "reopened");
}
