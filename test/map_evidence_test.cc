#include "navigation2d/exploration/map_evidence.h"

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <type_traits>

#include "navigation2d/exploration/doorway_geometry.h"
#include "navigation2d/exploration/live_route_safety.h"
using namespace navigation2d;
using namespace navigation2d::frontier_internal;
static_assert(!std::is_constructible_v<ObservationGridView, ExplorationGrid&&>);
static_assert(
    !std::is_constructible_v<PlanningGridView, ExplorationGrid&&, const ExplorationGrid&, double>);
static_assert(
    !std::is_constructible_v<PlanningGridView, const ExplorationGrid&, ExplorationGrid&&, double>);
ExplorationGrid Grid() {
  ExplorationGrid grid;
  grid.width = grid.height = 12;
  grid.resolution = .25;
  grid.cells.assign(144, 0);
  return grid;
}
int main() {
  auto grid = Grid();
  grid.cells[5 * 12 + 5] = 100;
  const PlanningGridView planning(grid, grid, .3);
  assert(!planning.ClearAt(4, 5, .125));
  assert(planning.ClearAt(4, 5, std::nextafter(.125, 0.)));
  assert(!planning.HasFreeLineOfSight(3, 5, 7, 5));
  assert(planning.HasFreeLineOfSight(3, 3, 7, 3));
  assert(OccupiedClearanceAt(grid, 4, 5) == .125);
  grid.cells[5 * 12 + 5] = -1;
  assert(planning.ClearAt(4, 5, .125));
  assert(!planning.Free(5, 5));
  auto observation = Grid();
  observation.cells[6 * 12 + 6] = -1;
  auto free = Grid();
  const PlanningGridView masked(free, observation, .3);
  assert(masked.Free(6, 6));
  assert(!masked.SafeViewpoint(6, 6));

  auto doorway = Grid();
  doorway.cells[5 * 12 + 6] = -1;
  doorway.cells[5 * 12 + 7] = -1;
  assert(!UnknownRoomBeyond(doorway, 5, 5));
  doorway.cells[5 * 12 + 8] = -1;
  assert(UnknownRoomBeyond(doorway, 5, 5));

  ExplorationGrid small;
  small.width = small.height = 3;
  small.resolution = .1;
  small.cells.assign(9, 0);
  small.cells[4] = -1;
  small.cells[0] = 100;
  const ObservationGridView raw(small);
  assert(raw.CountFrontiers() == 4);
  assert(raw.KnownCells() == 8);
  small.width = 0;
  assert(raw.CountFrontiers() == 0);
  assert(raw.KnownCells() == 8);

  MapEvidence maps;
  auto old = Grid();
  maps.Update(std::move(old), [](const auto&) {});
  auto next = Grid();
  next.origin_x = 8.;
  bool observed = false;
  try {
    maps.Update(std::move(next), [&](const auto& incoming) {
      observed = true;
      assert(incoming.origin_x == 8.);
      assert(maps.observation().origin_x == 8.);
      assert(maps.planning().origin_x == 0.);
      throw std::runtime_error("observer failed");
    });
    assert(false);
  } catch (const std::runtime_error&) {
  }
  assert(observed);
  assert(maps.observation().origin_x == 8.);
  assert(maps.planning().origin_x == 0.);
  maps.Update(std::move(next), [](const auto&) {});
  assert(maps.planning().origin_x == 8.);
  auto copy = maps;
  auto replacement = Grid();
  maps.Update(std::move(replacement), [](const auto&) {});
  assert(copy.observation().origin_x == 8.);
  assert(copy.planning().origin_x == 8.);

  auto live = Grid();
  live.cells[5 * 12 + 5] = 100;
  const std::vector<std::pair<double, double>> path{{.875, 1.375}, {1.875, 1.375}};
  assert(PathCutsLiveOccupied(live, path, 0., 0., .1, 0.));
  assert(!PathCutsLiveOccupied(live, path, 1.375, 1.375, .1, 2.));
  live.cells[5 * 12 + 5] = -1;
  assert(!PathCutsLiveOccupied(live, path, 0., 0., .1, 0.));
  assert(!PreferTipAbandonAfterPathCut(1, 0.));
  assert(PreferTipAbandonAfterPathCut(2, 1.));
}
