#include "navigation2d/application/exploration/forward_blocked.h"

#include <functional>
#include <source_location>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/exploration/frontier_explorer.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
void Require(bool value, std::source_location where = std::source_location::current()) {
  if (!value) {
    throw std::runtime_error("forward blocked contract at " + std::to_string(where.line()));
  }
}
struct Actions final : ForwardBlockedActions {
  EntryCoordinator& entry;
  std::vector<std::string> events;
  std::function<void()> on_peel, on_escape, on_demote, on_cancel;
  bool peel = false, escape = false;
  explicit Actions(EntryCoordinator& value) : entry(value) {}
  void CancelRecoveryNavigation() override {
    events.push_back("cancel");
    if (on_cancel) {
      on_cancel();
    }
  }
  void SelectAfterEntry() override { events.push_back("select"); }
  void PublishStatus(const std::string& phase, const std::string&) override {
    events.push_back(phase);
  }
  void AbandonLockedEntryMouth(const std::string&) override {
    events.push_back("abandon");
    entry.Clear();
  }
  bool EscapeOrShortRetreat(const char*, const char*) override {
    events.push_back("peel");
    if (on_peel) {
      on_peel();
    }
    return peel;
  }
  bool TryStartEscape() override {
    events.push_back("escape");
    if (on_escape) {
      on_escape();
    }
    return escape;
  }
  void DemoteViewpointForSeconds(const ExplorationGoal&, double seconds) override {
    Require(seconds == 8.);
    events.push_back("demote");
    if (on_demote) {
      on_demote();
    }
  }
};
}  // namespace
int main() {
  FrontierExplorer explorer;
  explorer.UpdateMap({30, 30, .1, 0., 0., std::vector<std::int8_t>(900, 0)});
  EntryCoordinator entry;
  GoalRecoverySession recovery;
  RetreatExecution retreat;
  TourExecution tour;
  ExplorationGoal goal;
  goal.x = goal.y = goal.frontier_x = goal.frontier_y = 1.;
  Actions actions(entry);
  const auto run = [&] {
    ExecuteForwardBlocked(entry, explorer, tour, recovery, retreat, goal, actions);
  };
  tour.RetryLater(goal);
  tour.RetryLater(goal);
  recovery.Begin(1., 1.);
  recovery.side_peels = 3;
  actions.on_demote = [&] {
    Require(explorer.failed_goals() == 1 && tour.goals().size() == 2 && recovery.active);
  };
  actions.on_escape = [&] { Require(tour.goals().size() == 1 && !recovery.active); };
  actions.escape = true;
  run();
  Require(actions.events == std::vector<std::string>{"demote", "escape", "cancel", "RECOVERING"});
  actions.events.clear();
  actions.on_demote = nullptr;
  actions.on_escape = nullptr;
  actions.escape = false;
  tour.Suspend();
  run();
  Require(actions.events ==
          std::vector<std::string>{"demote", "escape", "cancel", "select", "SELECTING"});
  // A pending entry takes the entry path even without a lock. No failure or
  // viewpoint/tour cooldown is charged before its bounded peel.
  actions.events.clear();
  entry.SetPending(goal);
  recovery.Begin(1., 1.);
  recovery.side_peels = 2;
  actions.peel = true;
  actions.on_peel = [&] { Require(entry.pending() && recovery.side_peels == 2); };
  const auto failures = explorer.failed_goals();
  run();
  Require(actions.events == std::vector<std::string>{"cancel", "peel"});
  Require(entry.pending() && explorer.failed_goals() == failures);
  actions.events.clear();
  actions.peel = false;
  run();
  Require(actions.events == std::vector<std::string>{"cancel", "peel", "abandon", "select"});
  Require(!entry.pending() && !entry.locked() && recovery.side_peels == 2);
  actions.events.clear();
  entry.Lock(goal, false, {123, 7});
  entry.CountAttempt();
  entry.CountAttempt();
  entry.ClearPending();
  actions.peel = true;
  actions.on_cancel = [&] { Require(entry.pending() && entry.pending()->x == goal.x); };
  run();
  Require(actions.events == std::vector<std::string>{"cancel", "peel"});
  Require(entry.locked() && entry.pending() && entry.attempts() == 2);
  Require(entry.evidence().known_cells == 123 && entry.evidence().map_revision == 7);
}
