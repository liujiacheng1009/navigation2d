#include "navigation2d/application/exploration/room_visit_execution.h"

#include <source_location>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/exploration/frontier_explorer.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
void Require(bool value, std::source_location location = std::source_location::current()) {
  if (!value) {
    throw std::runtime_error("room visit execution at " + std::to_string(location.line()));
  }
}
struct Actions final : RoomVisitActions {
  bool accept = false, fail = false;
  std::vector<std::string> events;
  ExplorationGoal requested;
  void PrepareRoomVisit() override { events.push_back("prepare"); }
  bool NavigateRoomVisit(const ExplorationGoal& goal) override {
    events.push_back("navigate");
    requested = goal;
    if (fail) {
      throw std::runtime_error("injected navigation exception");
    }
    return accept;
  }
  void ReportRoomVisit(const ExplorationGoal& goal, int attempt, bool accepted) override {
    Require(goal.x == requested.x && attempt >= 1 && attempt <= 2);
    events.push_back(accepted ? "accepted" : "rejected");
  }
};
}  // namespace
int main() {
  FrontierExplorer explorer;
  ExplorationGrid map{100, 100, .1, 0., 0., std::vector<std::int8_t>(10000, 0)};
  explorer.UpdateMap(map);
  RoomVisitLedger visits;
  Actions actions;
  const std::vector<std::pair<double, double>> trail{{1., 1.}};
  Require(!TryUnenteredRoomVisits(visits, explorer, trail, 1., 1., actions));
  Require(actions.events.empty());
  ExplorationGoal frontier, probe;
  probe.x = probe.y = 6.;
  visits.Remember(frontier, probe);
  actions.accept = true;
  Require(TryUnenteredRoomVisits(visits, explorer, trail, 1., 1., actions));
  Require(actions.events == std::vector<std::string>{"prepare", "navigate", "accepted"});
  Require(actions.requested.yaw > 0. && actions.requested.yaw < 1.6);
  actions.accept = false;
  actions.events.clear();
  Require(!TryUnenteredRoomVisits(visits, explorer, trail, 1., 1., actions));
  Require(actions.events == std::vector<std::string>{"prepare", "navigate", "rejected"});
  actions.events.clear();
  Require(!TryUnenteredRoomVisits(visits, explorer, trail, 1., 1., actions));
  Require(actions.events.empty());  // The same spot has consumed both attempts.
  RoomVisitLedger fault_visits;
  fault_visits.Remember(frontier, probe);
  actions.fail = true;
  bool threw = false;
  try {
    TryUnenteredRoomVisits(fault_visits, explorer, trail, 1., 1., actions);
  } catch (const std::runtime_error&) {
    threw = true;
  }
  Require(threw && fault_visits.TryVisit(actions.requested) == 2);
}
