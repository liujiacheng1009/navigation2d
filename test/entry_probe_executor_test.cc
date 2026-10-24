#include "navigation2d/application/exploration/entry_probe_executor.h"

#include "navigation2d/exploration/frontier_explorer.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/geometry/robot_geometry.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("entry execution contract failed");
  }
}
struct Actions final : EntryActions {
  bool cooling = false, accept = true;
  std::vector<std::string> trace;
  std::vector<EntryNotice> notices;
  ExplorationGoal target;
  const EntryCoordinator* state = nullptr;
  bool EntryMouthCooling(const ExplorationGoal&) override { return cooling; }
  void PrepareEntryPlan(double budget) override {
    trace.push_back(budget == 8. ? "prepare8" : "prepare4");
  }
  bool NavigateEntry(const ExplorationGoal& goal) override {
    target = goal;
    trace.push_back("navigate");
    return accept;
  }
  void CoolEnteredMouth(const ExplorationGoal&, double seconds) override {
    Require(seconds == 14. && state->pending().has_value());
    trace.push_back("cool");
  }
  void QueueActiveRegionFinish() override {
    Require(!state->pending() && !state->locked());
    trace.push_back("queue");
  }
  void MarkRegionFinishPending() override {
    throw std::runtime_error("unexpected driving completion");
  }
  void SelectAfterEntry() override { trace.push_back("select"); }
  void ReportEntry(const EntryNotice& notice) override {
    notices.push_back(notice);
    trace.push_back("report");
  }
};
FrontierExplorerConfig Config() {
  FrontierExplorerConfig config;
  config.minimum_frontier_cells = 6;
  config.footprint_clearance = kPlanningRadius();
  config.minimum_standoff = .10;
  config.maximum_standoff = 1.10;
  config.required_frontier_observations = 1;
  return config;
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
  ExplorationGoal boundary_a, boundary_b;
  boundary_b.x = .55;
  boundary_b.frontier_x = .75;
  Require(!EntryCoordinator::SameMouth(boundary_a, boundary_b));
  boundary_b.x = std::nextafter(.55, 0.);
  Require(EntryCoordinator::SameMouth(boundary_a, boundary_b));
  boundary_b.x = .55;
  boundary_b.frontier_x = std::nextafter(.75, 0.);
  Require(EntryCoordinator::SameMouth(boundary_a, boundary_b));
  EntryCoordinator entry;
  FrontierExplorer explorer(Config());
  RoomVisitLedger ledger;
  Actions actions;
  actions.state = &entry;
  EntryProbeObservation observation{.4, .7, 17, .18};
  auto run = [&] { return ExecuteEntryProbe(entry, explorer, ledger, actions, observation); };
  Require(run() == EntryProbeResult::kIdle && actions.trace.empty());
  ExplorationGoal far;
  far.x = 3.;
  far.frontier_x = 4.;
  far.frontier_y = .7;
  entry.SetPending(far);
  actions.cooling = true;
  Require(run() == EntryProbeResult::kCooling && !entry.locked() && actions.trace.empty());
  actions.cooling = false;
  Require(run() == EntryProbeResult::kDriving);
  Require(actions.trace == std::vector<std::string>{"prepare8", "navigate", "report"});
  Require(
      std::abs(std::hypot(actions.target.x - far.frontier_x, actions.target.y - far.frontier_y) -
               1.55) < 1e-12);
  Require(entry.attempts() == 0 && entry.evidence().map_revision == 17);
  Require(actions.notices.back().kind == EntryNoticeKind::kApproaching);

  // Failed approach continues into bounded target selection, consuming one
  // unavailable attempt rather than pretending navigation started.
  actions.trace.clear();
  actions.accept = false;
  Require(run() == EntryProbeResult::kUnavailable && entry.attempts() == 1);
  Require(actions.trace == std::vector<std::string>{"prepare8", "navigate", "report"});
  Require(actions.notices.back().kind == EntryNoticeKind::kUnavailable);

  explorer.UpdateMap(Door());
  auto goals = explorer.SelectGoals(.4, .7);
  Require(!goals.empty());
  auto mouth = goals.front();
  mouth.x = mouth.frontier_x - .45;
  mouth.y = mouth.frontier_y;
  entry.Clear();
  entry.SetPending(mouth);
  actions.trace.clear();
  Require(run() == EntryProbeResult::kRejected);
  Require(actions.trace == std::vector<std::string>{"prepare4", "navigate", "report"});
  Require(entry.attempts() == 1 && !ledger.empty());
  Require(actions.notices.back().kind == EntryNoticeKind::kPlanRejected);
  Require(entry.evidence().known_cells == explorer.KnownCells());
  actions.accept = true;
  actions.trace.clear();
  Require(run() == EntryProbeResult::kDriving && entry.attempts() == 2);
  Require(actions.notices.back().kind == EntryNoticeKind::kProbing);

  // Force the tolerance rejection independently of map geometry. Evidence and
  // room memory are committed before rejection; no navigation call is made.
  observation.goal_tolerance_m = 10.;
  actions.trace.clear();
  Require(run() == EntryProbeResult::kRejected && entry.attempts() == 3);
  Require(actions.trace == std::vector<std::string>{"prepare4", "report"});
  Require(actions.notices.back().kind == EntryNoticeKind::kTooNear);

  // The same real doorway, now on its interior side. The mouth is cleared
  // before room scheduling and reporting, without issuing a navigation goal.
  observation.robot_x = mouth.frontier_x + .55;
  observation.robot_y = mouth.frontier_y;
  observation.goal_tolerance_m = .18;
  actions.trace.clear();
  Require(run() == EntryProbeResult::kEntered);
  Require(actions.trace == std::vector<std::string>{"cool", "queue", "select", "report"});
  Require(entry.attempts() == 0 && ledger.empty());
  Require(actions.notices.back().kind == EntryNoticeKind::kEntered);
}
