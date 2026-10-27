#include "navigation2d/application/exploration/alcove_exit.h"

#include <functional>
#include <source_location>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/retreat_execution.h"
#include "navigation2d/application/exploration/route_failure_history.h"
#include "navigation2d/application/exploration/tour_execution.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
void Require(bool value, std::source_location where = std::source_location::current()) {
  if (!value) {
    throw std::runtime_error("alcove exit at " + std::to_string(where.line()));
  }
}
struct Actions final : AlcoveExitActions {
  EntryCoordinator& entry;
  RouteFailureHistory& routes;
  mutable std::vector<std::string> events;
  std::function<void()> on_escape;
  bool passable = true, probe = true, escape = true;
  Actions(EntryCoordinator& e, RouteFailureHistory& r) : entry(e), routes(r) {}
  void ReportUnfinishedPocket(std::size_t, bool frontier) override {
    Require(frontier);
    events.push_back("unfinished");
  }
  void ReportExhaustedPocket(const AlcoveObservation& observation) override {
    Require(observation.goal_distance_m == .1);
    events.push_back("exhausted");
  }
  void CancelRecoveryNavigation() override { events.push_back("cancel"); }
  bool PassableMouthGoal(const ExplorationGoal&) const override {
    Require(routes.size() == 0);
    events.push_back("passable");
    return passable;
  }
  void LockEntryMouth(const ExplorationGoal& goal) override {
    events.push_back("lock");
    entry.Lock(goal, false, {100, 7});
  }
  void PrepareDoorjambRetry() override { events.push_back("evidence"); }
  bool StartEntryProbe() override {
    events.push_back("probe");
    entry.CountAttempt();
    return probe;
  }
  bool EscapeOrShortRetreat(const char*, const char*) override {
    events.push_back("escape");
    if (on_escape) {
      on_escape();
    }
    return escape;
  }
  void SelectAfterEntry() override { events.push_back("select"); }
  void PublishStatus(const std::string& phase, const std::string&) override {
    events.push_back(phase);
  }
};
struct RetreatPort final : RetreatActions {
  bool RetreatPinched() const override { return false; }
  void ClearRetreatCooling() override {}
  void PrepareRetreatEvidence(bool) override {}
  bool NavigateRetreat(const ExplorationGoal&) override { return true; }
  void CancelRetreatEvidence() override {}
};
}  // namespace
int main() {
  FrontierExplorer explorer;
  ExplorationGrid map{100, 100, .1, 0., 0., std::vector<std::int8_t>(10000, 0)};
  map.cells[40 * 100 + 40] = -1;
  explorer.UpdateMap(map);
  ExplorationGoal goal;
  goal.x = goal.y = 2.;
  goal.frontier_x = 3.95;
  goal.frontier_y = 4.05;
  Require(explorer.GoalRegionStillFrontier(goal));
  EntryCoordinator entry;
  TourExecution tour;
  tour.RetryLater(goal);
  tour.RetryLater(goal);
  RetreatExecution retreat;
  RouteFailureHistory routes;
  routes.NoteFailure(42);
  Actions actions(entry, routes);
  const AlcoveObservation observation{{1., 1., 0.}, .1, .1, .1, .5, .6};
  const auto run = [&] {
    ExecuteAlcoveExit(entry, explorer, tour, retreat, routes, goal, observation, actions);
  };
  run();
  Require(actions.events == std::vector<std::string>{"unfinished", "cancel", "passable", "lock",
                                                     "evidence", "probe"});
  Require(entry.pending() && entry.attempts() == 1 && tour.goals().size() == 2);
  Require(explorer.failed_goals() == 0 && routes.failures_at_pose() == 1);
  actions.events.clear();
  actions.passable = false;
  run();
  Require(actions.events ==
          std::vector<std::string>{"unfinished", "cancel", "passable", "select", "SELECTING"});
  Require(explorer.failed_goals() == 1 && tour.goals().size() == 1 && entry.pending());
  // With an exhausted map the entry, tour and route exclusions are cleared
  // before escape is requested, and no extra selection/status follows success.
  map.cells[40 * 100 + 40] = 0;
  explorer.UpdateMap(map);
  actions.events.clear();
  routes.Cool(99);
  actions.on_escape = [&] {
    Require(!entry.pending() && !entry.locked() && tour.goals().empty());
    Require(routes.size() == 0 && !retreat.active());
  };
  run();
  Require(actions.events == std::vector<std::string>{"exhausted", "cancel", "escape"});
  Require(explorer.completed_goals() == 1 && routes.failures_at_pose() == 1);
  actions.events.clear();
  actions.escape = false;
  run();
  Require(actions.events ==
          std::vector<std::string>{"exhausted", "cancel", "escape", "select", "SELECTING"});
  // Finishing an active retreat must not count another viewpoint as observed.
  RetreatPort retreat_port;
  std::vector<std::pair<double, double>> trail{{0., 0.}, {.5, 0.}, {1., 0.}, {1.5, 0.}};
  Require(retreat.Start(trail, {1.5, 0., 0.}, {15, 0}, false, retreat_port));
  const auto completed = explorer.completed_goals();
  actions.events.clear();
  run();
  Require(!retreat.active() && explorer.completed_goals() == completed);
}
