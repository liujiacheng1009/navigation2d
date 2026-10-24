#include <algorithm>

#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/test/completion_execution_fixture.h"
using namespace navigation2d;
using namespace navigation2d::exploration;
using namespace navigation2d::test;
int main() {
  {
    FrontierExplorer explorer;
    MissionLifecycle mission;
    TourExecution tour;
    tour.RetryLater(ExplorationGoal{.x = 9.});
    CompletionExecution execution;
    CompletionRecorder actions(mission, tour);
    execution.Start(explorer, mission, tour, {}, actions);
    assert(actions.events ==
           std::vector<std::string>({"clear_routes", "notice", "pinch", "wait", "return"}));
    assert(!actions.partial && actions.notices.front().rounds == 1);
    assert(mission.phase() == MissionPhase::kExploring && tour.goals().empty());
    assert(execution.CanProbe() && execution.ShouldWait(true, 99));
    execution.Start(explorer, mission, tour, {}, actions);
    execution.Start(explorer, mission, tour, {}, actions);
    assert(!execution.CanProbe());
    assert(execution.ShouldWait(true, 7));
    assert(!execution.ShouldWait(true, 8) && !execution.ShouldWait(false, 0));
    execution.ResetBudget();
    assert(execution.CanProbe());
  }
  {
    FrontierExplorer explorer;
    MissionLifecycle mission;
    TourExecution tour;
    CompletionExecution execution;
    CompletionRecorder actions(mission, tour);
    actions.pinch = actions.rescue = true;
    execution.Start(explorer, mission, tour, {}, actions);
    assert(actions.events ==
           std::vector<std::string>({"clear_routes", "notice", "pinch", "rescue"}));
  }
  {
    FrontierExplorer explorer;
    MissionLifecycle mission;
    TourExecution tour;
    CompletionExecution execution;
    CompletionRecorder actions(mission, tour);
    actions.waiting = true;
    execution.Start(explorer, mission, tour, {}, actions);
    assert(actions.events.back() == "unresolved boundary; waiting for map before return");
    assert(std::find(actions.events.begin(), actions.events.end(), "return") ==
           actions.events.end());
  }
  for (int mode = 0; mode < 4; ++mode) {
    FrontierExplorerConfig config;
    config.footprint_clearance = .3;
    FrontierExplorer explorer(config);
    explorer.UpdateMap(CompletionMap());
    MissionLifecycle mission;
    TourExecution tour;
    CompletionExecution execution;
    CompletionRecorder actions(mission, tour);
    actions.navigate = mode == 0;
    actions.thrash = mode == 1;
    actions.entry = mode == 2;
    actions.waiting = true;
    execution.Start(explorer, mission, tour, {1.2, 1.5, 0.}, actions);
    assert(std::find(actions.events.begin(), actions.events.end(), "navigate") !=
           actions.events.end());
    if (mode == 0) {
      assert(actions.events == std::vector<std::string>({"clear_routes", "navigate"}));
    } else if (mode == 1) {
      assert(actions.events == std::vector<std::string>({"clear_routes", "navigate", "thrash"}));
    } else if (mode == 2) {
      assert(actions.events == std::vector<std::string>({"clear_routes", "navigate", "thrash",
                                                         "lock", "entry", "notice"}));
      assert(actions.notices.back().kind == CompletionNoticeKind::kEntryProbe);
    } else {
      assert(actions.lock_clears > 0);
      assert(explorer.failed_goals() == actions.lock_clears);
      assert(actions.events.back() == "leftover blocked; waiting for map update before return");
    }
    assert(mission.phase() == (mode < 3 ? MissionPhase::kCompleting : MissionPhase::kExploring));
    if (mode < 3) {
      assert(explorer.failed_goals() == 0);
    }
  }
  {
    FrontierExplorerConfig config;
    config.footprint_clearance = .3;
    FrontierExplorer explorer(config);
    explorer.UpdateMap(CompletionMap());
    MissionLifecycle mission;
    TourExecution tour;
    CompletionExecution execution;
    CompletionRecorder actions(mission, tour);
    actions.throw_navigation = true;
    try {
      execution.Start(explorer, mission, tour, {1.2, 1.5, 0.}, actions);
      assert(false);
    } catch (const std::runtime_error&) {
    }
    assert(actions.events == std::vector<std::string>({"clear_routes", "navigate"}));
    assert(mission.phase() == MissionPhase::kCompleting);
    assert(explorer.failed_goals() == 0 && actions.lock_clears == 0);
  }
  for (bool exhausted : {false, true}) {
    FrontierExplorerConfig config;
    config.minimum_frontier_cells = 100;
    config.footprint_clearance = .3;
    FrontierExplorer explorer(config), empty;
    auto map = CompletionMap();
    map.cells.assign(1500, 0);
    map.cells[15 * 50 + 25] = -1;
    explorer.UpdateMap(map);
    explorer.SelectGoals(1.2, 1.5);
    assert(explorer.raw_frontier_cells() > 0);
    assert(explorer.candidate_frontier_goals() == 0);
    MissionLifecycle mission;
    TourExecution tour;
    CompletionExecution execution;
    CompletionRecorder actions(mission, tour);
    if (exhausted) {
      for (int i = 0; i < 3; ++i) {
        execution.Start(empty, mission, tour, {}, actions);
      }
      actions.events.clear();
    }
    execution.Finish(explorer, mission, tour, actions);
    assert(actions.events == (exhausted ? std::vector<std::string>({"build", "wait", "return"})
                                        : std::vector<std::string>({"build", "probe_again"})));
    if (exhausted) {
      assert(actions.partial);
    }
  }
  for (bool seed : {false, true}) {
    FrontierExplorer explorer;
    MissionLifecycle mission;
    TourExecution tour;
    tour.RetryLater(ExplorationGoal{.x = 9.});
    CompletionExecution execution;
    CompletionRecorder actions(mission, tour);
    actions.seed_tour = seed;
    execution.Finish(explorer, mission, tour, actions);
    assert(actions.events == (seed ? std::vector<std::string>({"build", "notice", "select"})
                                   : std::vector<std::string>({"build", "wait", "return"})));
    assert(mission.phase() == MissionPhase::kExploring);
    assert(tour.goals().size() == static_cast<std::size_t>(seed));
  }
}
