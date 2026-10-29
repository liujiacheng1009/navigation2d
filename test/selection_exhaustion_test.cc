#include <algorithm>

#include "navigation2d/application/exploration/completion_execution.h"
#include "navigation2d/application/exploration/tour_execution.h"
#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/test/completion_execution_fixture.h"
#include "navigation2d/test/selection_exhaustion_fixture.h"
namespace {
using namespace navigation2d;
ExplorationGrid Map(int width = 100, int height = 60) {
  return {width, height, .05, 0., 0., std::vector<std::int8_t>(width * height, 0)};
}
}  // namespace
int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  using namespace navigation2d::test;
  FrontierExplorerConfig config;
  config.cover_residuals = false;
  FrontierExplorer explorer(config);
  explorer.UpdateMap(Map());
  TourExecution tour;
  CompletionExecution completion;
  for (int budget : {2, 3, 6}) {
    SelectionExhaustion selection;
    selection.InitializeKnown(explorer.KnownCells());
    SelectionRecorder actions;
    actions.pinched = budget == 2;
    actions.swing_clear = budget == 3;
    for (int cycle = 1; cycle <= budget; ++cycle) {
      actions.events.clear();
      selection.Resolve(explorer, completion, tour, {{1.2, 1.5, 0.}, 1}, actions);
      assert(actions.events.front() == "stop");
      assert(explorer.KnownCells() == 6000 && selection.blocked_cycles() == cycle);
      assert(actions.returns == (cycle == budget ? 1 : 0));
      assert(actions.partial == (cycle == budget));
      assert(actions.probes == 0);
    }
    selection.ResetBlocked();
    assert(selection.blocked_cycles() == 0);
  }
  // No attempted plans: preserve lazy range sensing (the six-cycle path).
  {
    SelectionExhaustion selection;
    SelectionRecorder actions;
    tour.RetryLater(ExplorationGoal{});
    selection.Resolve(explorer, completion, tour, {{1.2, 1.5, 0.}, 0}, actions);
    assert(std::find(actions.events.begin(), actions.events.end(), "swing") ==
           actions.events.end());
    tour.Suspend();
  }
  // New evidence requires 500 cells; single-cell noise does not reset retry budgets.
  {
    SelectionExhaustion selection;
    SelectionRecorder actions;
    selection.InitializeKnown(6000);
    selection.Resolve(explorer, completion, tour, {{1.2, 1.5, 0.}, 1}, actions);
    auto map = Map(100, 65);
    map.cells.back() = -1;  // 6499 known, one below the meaningful-growth bound.
    explorer.UpdateMap(map);
    selection.Resolve(explorer, completion, tour, {{1.2, 1.5, 0.}, 1}, actions);
    assert(selection.blocked_cycles() == 2);
    map.cells.back() = 0;
    explorer.UpdateMap(map);
    selection.Resolve(explorer, completion, tour, {{1.2, 1.5, 0.}, 1}, actions);
    assert(selection.blocked_cycles() == 1);
  }
  // Meaningful new evidence reopens an exhausted residual-probe budget.
  {
    MissionLifecycle mission;
    CompletionRecorder completing(mission, tour);
    for (int i = 0; i < 3; ++i) {
      completion.Start(explorer, mission, tour, {1.2, 1.5, 0.}, completing);
    }
    assert(!completion.CanProbe());
    SelectionExhaustion selection;
    SelectionRecorder actions;
    selection.InitializeKnown(explorer.KnownCells() - 499);
    selection.Resolve(explorer, completion, tour, {{1.2, 1.5, 0.}, 1}, actions);
    assert(!completion.CanProbe());
    selection.InitializeKnown(explorer.KnownCells() - 500);
    selection.Resolve(explorer, completion, tour, {{1.2, 1.5, 0.}, 1}, actions);
    assert(completion.CanProbe() && selection.blocked_cycles() == 1);
  }
  // Bootstrap crops must keep waiting; a mature map returns after 12 empty selections.
  {
    SelectionExhaustion selection;
    SelectionRecorder actions;
    for (int i = 0; i < 15; ++i) {
      selection.Resolve(explorer, completion, tour, {{}, 0}, actions);
    }
    assert(actions.direct_returns == 0);
    selection.GoalAccepted();
    explorer.UpdateMap(Map(250, 100));
    for (int i = 0; i < 11; ++i) {
      selection.Resolve(explorer, completion, tour, {{}, 0}, actions);
    }
    assert(actions.direct_returns == 0);
    selection.Resolve(explorer, completion, tour, {{}, 0}, actions);
    assert(actions.direct_returns == 1 && selection.blocked_cycles() == 0);
  }
  // Raw residuals with no admitted mouth must still probe, rescue, wait or
  // return according to the residual budget; they never fall into generic clearing.
  for (int mode = 0; mode < 4; ++mode) {
    FrontierExplorerConfig residual_config;
    residual_config.minimum_frontier_cells = 100;
    FrontierExplorer residuals(residual_config), empty;
    auto map = CompletionMap();
    map.cells.assign(1500, 0);
    map.cells[15 * 50 + 25] = -1;
    residuals.UpdateMap(map);
    residuals.SelectGoals(1.2, 1.5);
    assert(residuals.ShouldCoverResiduals() && residuals.candidate_frontier_goals() == 0);
    CompletionExecution budget;
    MissionLifecycle mission;
    CompletionRecorder completing(mission, tour);
    if (mode > 0) {
      for (int i = 0; i < 3; ++i) {
        budget.Start(empty, mission, tour, {}, completing);
      }
    }
    SelectionExhaustion selection;
    selection.InitializeKnown(residuals.KnownCells());
    SelectionRecorder actions;
    actions.pinched = actions.rescue = mode == 2;
    actions.wait = mode == 3;
    selection.Resolve(residuals, budget, tour, {{1.2, 1.5, 0.}, 0}, actions);
    assert(actions.events.front() == "stop");
    assert(std::find(actions.events.begin(), actions.events.end(), "clear") ==
           actions.events.end());
    assert(actions.probes == (mode == 0 ? 1 : 0));
    assert(actions.returns == (mode == 1 ? 1 : 0));
    assert(actions.partial == (mode == 1));
    if (mode == 2) {
      assert(actions.events.back() == "rescue");
    }
    if (mode == 3) {
      assert(actions.events.back() == "openable unknown remains; waiting for map update");
    }
  }
  // An effect failure propagates before budget/state advancement.
  {
    SelectionExhaustion selection;
    SelectionRecorder actions;
    actions.throw_stop = true;
    bool threw = false;
    try {
      selection.Resolve(explorer, completion, tour, {{}, 1}, actions);
    } catch (const std::runtime_error&) {
      threw = true;
    }
    assert(threw && actions.events == std::vector<std::string>{"stop"});
    assert(selection.blocked_cycles() == 0);
  }
}
