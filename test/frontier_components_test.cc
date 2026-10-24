#include "navigation2d/exploration/frontier_components.h"

#include <cassert>
#include <cmath>
#include <vector>

#include "navigation2d/exploration/frontier_visibility.h"
int main() {
  using namespace navigation2d;
  using namespace navigation2d::frontier_internal;
  ExplorationGrid map{5, 5, .25, -1., -1., std::vector<std::int8_t>(25, -1)};
  map.cells[6] = map.cells[12] = map.cells[24] = 0;
  std::vector<int> travel(25, -1);
  travel[6] = 0;
  FrontierComponents components(5, 5);
  assert(components.Discover(map, travel) == 2);
  const auto first = components.Next();
  assert(first && *first == std::vector<int>({6, 12}));
  assert(!components.Next() && !components.Next());
  const auto anchor = LocateCluster(*first, map);
  assert(anchor.representative == 6 && anchor.mean_col == 1.5 && anchor.mean_row == 1.5);
  assert(anchor.frontier_x == -.625 && anchor.frontier_y == -.625);
  // Discover resets marks/cursor; an unreachable component is never included.
  travel.assign(25, -1);
  travel[24] = 0;
  assert(components.Discover(map, travel) == 1);
  const auto second = components.Next();
  assert(second && *second == std::vector<int>{24});
  assert(!components.Next());
  travel.assign(25, -1);
  travel[13] = 0;
  assert(components.Discover(map, travel) == 1);
  const auto third = components.Next();
  assert(third && *third == std::vector<int>{12});
  // The carved travel cell itself is unknown evidence and cannot be a boundary.
  assert(!components.Next());
  travel.assign(25, -1);
  assert(components.Discover(map, travel) == 0 && !components.Next());
  ExplorationGrid view{9, 9, .1, 0., 0., std::vector<std::int8_t>(81, -1)};
  view.cells[40] = 0;
  const double gain = VisibleUnknownGain(view, .1, 4, 4);
  assert(gain > 0.);
  view.resolution = .2;
  assert(std::abs(VisibleUnknownGain(view, .1, 4, 4) - 4. * gain) < 1e-12);
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx != 0 || dy != 0) {
        view.cells[(4 + dy) * 9 + 4 + dx] = 100;
      }
    }
  }
  assert(VisibleUnknownGain(view, .1, 4, 4) == 0.);
  view.cells.assign(81, 0);
  assert(VisibleUnknownGain(view, .1, 4, 4) == 0.);
}
