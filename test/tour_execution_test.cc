#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/application/exploration/tour_execution.h"

#include <cstdint>
#include <deque>
#include <initializer_list>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("tour execution contract failed");
  }
}
ExplorationGoal GoalAt(double x) {
  ExplorationGoal goal;
  goal.x = goal.frontier_x = x;
  return goal;
}
struct Constraints final : TourConstraints {
  int cooling_queries = 0;
  mutable int direction_queries = 0;
  bool RecentlyDemoted(const ExplorationGoal& goal) override {
    ++cooling_queries;
    return goal.x == 3.;
  }
  bool GoalIsBehindTrail(const ExplorationGoal& goal) const override {
    ++direction_queries;
    return goal.x < 0.;
  }
};
void Sequence(const TourExecution& execution, std::initializer_list<double> expected) {
  Require(execution.goals().size() == expected.size());
  auto next = execution.goals().begin();
  for (double x : expected) {
    Require((next++)->x == x);
  }
}
void Seed(TourExecution& execution) {
  for (double x : {-2., 8., 2., 3., 4., 0.}) {
    execution.RetryLater(GoalAt(x));
  }
}
ExplorationGrid Door() {
  ExplorationGrid map{50, 30, .05, 0., 0., std::vector<std::int8_t>(1500, 100)};
  for (int row = 4; row < 26; ++row) {
    for (int col = 2; col < 16; ++col) {
      map.cells[row * 50 + col] = 0;
    }
    for (int col = 21; col < 29; ++col) {
      map.cells[row * 50 + col] = 0;
    }
  }
  for (int row = 8; row < 22; ++row) {
    for (int col = 16; col < 21; ++col) {
      map.cells[row * 50 + col] = -1;
    }
  }
  return map;
}
}  // namespace
int main() {
  static_assert(std::is_same_v<decltype(std::declval<TourExecution&>().goals()),
                               const std::deque<ExplorationGoal>&>);
  FrontierExplorer explorer;
  TourExecution execution;
  Constraints constraints;
  Seed(execution);
  execution.AfterMouthFailure(explorer, GoalAt(0.), std::nullopt, constraints);
  // Ahead-local, existing far, behind-local. Exactly 4 m is still local;
  // the abandoned mouth is excluded before querying the cooldown clock.
  Sequence(execution, {2., 4., 8., -2.});
  Require(constraints.cooling_queries == 5 && constraints.direction_queries == 3);
  execution.Suspend();
  Seed(execution);
  constraints = {};
  execution.AfterWallTip(explorer, GoalAt(0.), std::nullopt, constraints);
  // Wall-tip recovery keeps original local order and does not query trail
  // direction or active-room membership as the mouth policy does.
  Sequence(execution, {-2., 2., 4., 8.});
  Require(constraints.cooling_queries == 5 && constraints.direction_queries == 0);
  execution.DeferCurrent(execution.goals().front());
  Sequence(execution, {2., 4., 8., -2.});
  execution.HandoffThrough(2);
  Sequence(execution, {8., -2.});
  execution.Advance();
  Sequence(execution, {-2.});
  execution.Suspend();
  Require(execution.revision() == 0);

  // An empty valid map produces no tasks. Revision advances on Build only,
  // while room finishing and phase suspension leave that revision intact.
  ExplorationGrid map{10, 10, .1, 0., 0., std::vector<std::int8_t>(100, 100)};
  explorer.UpdateMap(map);
  execution.Build(explorer, {.5, .5, 0.});
  Sequence(execution, {});
  Require(execution.revision() == 1);
  execution.QueueActiveRoom(explorer, {.5, .5, 0.}, constraints);
  Require(execution.revision() == 1);
  execution.RetryLater(GoalAt(2.));
  execution.SkipResolved(explorer);
  Sequence(execution, {});
  execution.Build(explorer, {.5, .5, 0.});
  execution.Suspend();
  Require(execution.revision() == 2);

  // Distant rescue applies different legacy cutoffs to committed tasks and
  // leftovers. No map frontiers here: assert retained task order and cooling.
  for (double x : {1.79, 1.8, 3., 4.}) {
    execution.RetryLater(GoalAt(x));
  }
  Require(execution.PrioritizeDistant(explorer, {0., 0., 0.}, constraints));
  Sequence(execution, {1.8, 4.});
  // Discovery runs again during reordering. All already queued real mouths
  // must remain unique when the map and candidate geometry are unchanged.
  FrontierExplorerConfig config;
  config.minimum_frontier_cells = 6;
  config.minimum_standoff = .10;
  config.maximum_standoff = 1.10;
  config.required_frontier_observations = 1;
  FrontierExplorer door(config);
  door.UpdateMap(Door());
  const auto mouths = door.SelectGoals(.4, .7);
  Require(!mouths.empty());
  execution.Suspend();
  for (const auto& mouth : mouths) {
    execution.RetryLater(mouth);
  }
  const auto count = execution.goals().size();
  execution.AfterMouthFailure(door, GoalAt(-10.), TourPose{.4, .7, 0.}, constraints);
  Require(execution.goals().size() == count);
  execution.AfterWallTip(door, GoalAt(-10.), TourPose{.4, .7, 0.}, constraints);
  Require(execution.goals().size() == count);
}
