#include <cassert>
#include <vector>

#include "navigation2d/planning/incremental_search.h"
#include "navigation2d/planning/search_core.h"

int main() {
  using navigation2d::planning_internal::BestFirstSearch;
  using navigation2d::planning_internal::RestoreStatePath;
  using navigation2d::planning_internal::SearchOptions;
  // The generic core is independent of grid geometry and deterministically
  // chooses state 1 when two equal-cost branches are inserted in that order.
  const auto graph = BestFirstSearch(
      4, 0, [](int state) { return state == 3; }, [](int) { return 0.; },
      [](int state, int, const auto& visit) {
        if (state == 0) {
          visit({1, 1.});
          visit({2, 1.});
        }
        if (state == 1 || state == 2) {
          visit({3, 1.});
        }
      });
  assert(graph.found);
  assert((RestoreStatePath(graph, 0, 3) == std::vector<int>{0, 1, 3}));
  SearchOptions limited;
  limited.max_expansions = 1;
  const auto truncated = BestFirstSearch(
      4, 0, [](int state) { return state == 3; }, [](int) { return 0.; },
      [](int state, int, const auto& visit) {
        if (state == 0) {
          visit({1, 1.});
        }
      },
      limited);
  assert(!truncated.found && truncated.diagnostics.expansion_limit_reached);
  const auto anytime = navigation2d::planning_internal::AnytimeRepairingAStar(
      5, 0, 4, [](int state) { return state == 4 ? 0. : 1.; },
      [](int state, int, const auto& visit) {
        if (state == 0) {
          visit({1, 4.});
          visit({2, 1.});
        }
        if (state == 1) {
          visit({4, 1.});
        }
        if (state == 2) {
          visit({3, 1.});
        }
        if (state == 3) {
          visit({4, 1.});
        }
      },
      2.5, .5);
  assert(anytime.found && anytime.suboptimality_bound == 1.);
  assert(anytime.cost_to_come[4] == 3.);
  std::vector<std::vector<std::pair<int, double>>> incremental_edges{
      {{1, 1.}, {2, 2.}}, {{3, 1.}}, {{3, 1.}}, {}};
  const auto successors = [&](int state, const auto& visit) {
    for (const auto& [next, cost] : incremental_edges[state]) {
      visit({next, cost});
    }
  };
  const auto predecessors = [&](int state, const auto& visit) {
    for (int source = 0; source < 4; ++source) {
      for (const auto& [next, cost] : incremental_edges[source]) {
        if (next == state) {
          visit({source, cost});
        }
      }
    }
  };
  navigation2d::planning_internal::DStarLite incremental(4, 3, successors, predecessors,
                                                         [](int, int) { return 0.; });
  auto incremental_result = incremental.Plan(0, {});
  assert(incremental_result.found);
  assert((incremental_result.path == std::vector<int>{0, 1, 3}));
  incremental_edges[1][0].second = 10.;
  incremental_result = incremental.Plan(0, {1});
  assert(incremental_result.found && incremental_result.reused);
  assert((incremental_result.path == std::vector<int>{0, 2, 3}));
  incremental_result = incremental.Plan(2, {});
  assert((incremental_result.path == std::vector<int>{2, 3}));
}
