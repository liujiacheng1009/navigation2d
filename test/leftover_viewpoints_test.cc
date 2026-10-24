#include "navigation2d/exploration/leftover_viewpoints.h"

#include <source_location>
#include <stdexcept>
#include <string>
namespace {
void Require(bool value, std::source_location location = std::source_location::current()) {
  if (!value) {
    throw std::runtime_error("leftover viewpoint contract at " + std::to_string(location.line()));
  }
}
}  // namespace
int main() {
  using namespace navigation2d;
  using namespace navigation2d::frontier_internal;
  ExplorationGrid map{40, 40, .05, 0., 0., std::vector<std::int8_t>(1600, 0)};
  TipLedger tips;
  FrontierVisitMemory visits;
  const auto query = [&](int count) {
    return SelectLeftoverViewpoints({map, map, .1, tips, visits}, .5, .5, count);
  };
  Require(!query(0).raw_frontier_cells);
  auto result = query(2);
  Require(result.raw_frontier_cells && *result.raw_frontier_cells == 0 && result.goals.empty());
  map.cells[20 * 40 + 20] = -1;
  result = query(2);
  Require(result.raw_frontier_cells && *result.raw_frontier_cells == 4);
  Require(result.goals.size() == 1 && result.goals[0].leftover_approach);
  Require(result.goals[0].frontier_cells == 1 && result.goals[0].information_gain == .05);
  visits.Add({1., 1., .35, 0, 10000, false});
  Require(!query(2).goals.empty());  // Narrow temporary failure cooling is ignored.
  visits.Add({1., 1., 1.5, 0, 10000, false});
  Require(query(2).goals.empty());  // Wide pocket cooling applies to completion.
  visits.Clear();
  for (int i = 0; i < 3; ++i) {
    tips.NoteUnreached(1., 1., false);
  }
  Require(query(2).goals.empty());
  auto blocked = map;
  blocked.cells.assign(1600, 100);
  result = SelectLeftoverViewpoints({blocked, map, .1, tips, visits}, .5, .5, 2);
  Require(!result.raw_frontier_cells && result.goals.empty());
}
