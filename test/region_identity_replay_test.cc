#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/exploration/region_scheduler.h"

int main() {
  using namespace navigation2d;
  RegionScheduler scheduler;
  for (int update = 0; update < 160; ++update) {
    ExplorationGrid grid;
    grid.width = 34;
    grid.height = 14;
    grid.resolution = update < 80 ? .1 : .2;
    const int padding = update % 3;
    grid.origin_x = -padding * grid.resolution;
    grid.cells.assign(grid.width * grid.height, 100);
    for (int row = 2; row <= 10; ++row) {
      for (int col = 2; col <= 12; ++col) {
        grid.cells[row * grid.width + col + padding] = 0;
      }
      for (int col = 20; col <= 28; ++col) {
        grid.cells[row * grid.width + col + padding] = 0;
      }
    }
    if (update % 4 == 1 || update % 4 == 2) {
      for (int col = 13; col <= 19; ++col) {
        grid.cells[6 * grid.width + col + padding] = 0;
      }
    }
    // A temporarily absent speck is remembered; a long disappearance ages
    // its ownership out. Unknown borders also exercise evidence changes.
    if (update < 12 || update >= 100) {
      grid.cells[grid.width + 16 + padding] = 0;
    }
    if (update % 5 == 0) {
      grid.cells[2 * grid.width + 13 + padding] = -1;
    }
    scheduler.Update(grid, .2);
    if (update % 7 == 0) {
      scheduler.Enter(6.5 * grid.resolution, 6.5 * grid.resolution);
    }
    std::uint64_t raster = 1469598103934665603ULL;
    for (int row = 0; row < grid.height; ++row) {
      for (int col = 0; col < grid.width; ++col) {
        const auto id = scheduler.RegionAt(grid.origin_x + (col + .5) * grid.resolution,
                                           (row + .5) * grid.resolution);
        assert(!id || scheduler.regions().contains(id));
        raster = (raster ^ id) * 1099511628211ULL;
      }
    }
    assert(scheduler.RegionAt(NAN, 0.) == 0);
    assert(scheduler.RegionAt(-100., -100.) == 0);
    std::cout << update << ' ' << raster << ' ' << scheduler.Complete() << ' ' << scheduler.Audit();
    for (const auto& [id, region] : scheduler.regions()) {
      std::cout << ' ' << id << ':' << region.parent << ':' << region.visible << ':'
                << region.evidence << ':' << region.free_cells << ':' << region.frontier_cells;
      for (const auto neighbor : region.neighbors) {
        std::cout << ',' << neighbor;
      }
    }
    std::cout << '\n';
    // Invalid observations must leave identity, graph and DFS commitment intact.
    auto invalid = grid;
    invalid.cells.pop_back();
    const auto before = scheduler.Audit();
    scheduler.Update(invalid, .2);
    assert(scheduler.Audit() == before);
  }
}
