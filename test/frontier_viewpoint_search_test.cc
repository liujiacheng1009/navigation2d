#include "navigation2d/exploration/frontier_viewpoint_search.h"

#include <cassert>
#include <cmath>
#include <vector>

int main() {
  using namespace navigation2d;
  using namespace navigation2d::frontier_internal;
  ExplorationGrid map{40, 30, .1, 0., 0., std::vector<std::int8_t>(1200, 0)};
  auto observation = map;
  for (int row = 0; row < 30; ++row) {
    for (int col = 21; col < 40; ++col) {
      observation.cells[row * 40 + col] = -1;
    }
  }
  FrontierExplorerConfig config;
  FrontierVisitMemory visits;
  std::vector<unsigned char> safe(1200, 0);
  std::vector<int> travel(1200, -1);
  std::vector<int> cluster{15 * 40 + 20};
  auto anchor = LocateCluster(cluster, map);
  const int near = 15 * 40 + 14;
  safe[near] = 1;
  travel[near] = 10;
  const auto problem = [&] {
    return FrontierViewpointProblem{map,  observation, config,  visits,
                                    safe, travel,      cluster, anchor};
  };
  auto direct = SearchVisibleViewpoint(problem(), false);
  assert(direct && std::abs(direct->x - 1.45) < 1e-12 && std::abs(direct->y - 1.55) < 1e-12);
  auto selected = SelectComponentViewpoint(problem());
  assert(selected && selected->score == direct->score);
  assert(selected->information_gain > 0. && selected->frontier_cells == 1);
  // Painted planning free does not authorize a viewpoint in raw unknown.
  observation.cells[near] = -1;
  assert(!SelectComponentViewpoint(problem()));
  observation.cells[near] = 0;
  travel[near] = -1;
  assert(!SelectComponentViewpoint(problem()));
  travel[near] = 10;
  safe[near] = 0;
  assert(!SelectComponentViewpoint(problem()));
  safe[near] = 1;
  visits.Add({1.45, 1.55, .2, 0, 600, true});
  assert(!SelectComponentViewpoint(problem()));
  visits.Clear();
  // Bounded open approach is last resort when standoff and corridor tests fail.
  safe[near] = 0;
  const int far = 15 * 40 + 5;
  safe[far] = 1;
  travel[far] = 10;
  assert(!SearchVisibleViewpoint(problem(), false));
  assert(!SearchVisibleViewpoint(problem(), true));
  assert(!SearchNarrowViewpoint(problem()));
  const auto open = SearchOpenApproach(problem());
  selected = SelectComponentViewpoint(problem());
  assert(open && selected && open->score == selected->score);
  assert(std::abs(selected->x - .55) < 1e-12);
  // A narrow approach survives occluded representative/member LOS.
  safe[far] = 0;
  safe[near] = 1;
  map.cells.assign(1200, 100);
  for (int row = 12; row <= 18; ++row) {
    for (int col = 0; col < 40; ++col) {
      map.cells[row * 40 + col] = 0;
    }
  }
  map.cells[15 * 40 + 18] = 100;
  assert(!SearchVisibleViewpoint(problem(), false));
  assert(!SearchVisibleViewpoint(problem(), true));
  const auto narrow = SearchNarrowViewpoint(problem());
  selected = SelectComponentViewpoint(problem());
  assert(narrow && selected && narrow->score == selected->score);
  // Any-member LOS precedes corridor fallback, while component identity stays
  // at the first centroid-tied member (the occluded representative).
  cluster.push_back(17 * 40 + 20);
  anchor = LocateCluster(cluster, map);
  assert(anchor.representative == cluster.front());
  assert(!SearchVisibleViewpoint(problem(), false));
  const auto member = SearchVisibleViewpoint(problem(), true);
  selected = SelectComponentViewpoint(problem());
  assert(member && selected && member->score == selected->score);
  assert(std::abs(selected->frontier_x - 2.05) < 1e-12 &&
         std::abs(selected->frontier_y - 1.55) < 1e-12);
}
