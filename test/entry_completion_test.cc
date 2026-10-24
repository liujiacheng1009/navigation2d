#include "navigation2d/application/exploration/entry_completion.h"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("entry completion contract failed");
  }
}
struct Actions final : EntryActions {
  const EntryCoordinator& entry;
  const RoomVisitLedger& ledger;
  EntryArrival arrival;
  std::vector<std::string> trace;
  Actions(const EntryCoordinator& state, const RoomVisitLedger& visits, EntryArrival mode)
      : entry(state), ledger(visits), arrival(mode) {}
  bool EntryMouthCooling(const ExplorationGoal&) override {
    throw std::logic_error("unexpected query");
  }
  void PrepareEntryPlan(double) override { throw std::logic_error("unexpected preparation"); }
  bool NavigateEntry(const ExplorationGoal&) override {
    throw std::logic_error("unexpected navigation");
  }
  void CoolEnteredMouth(const ExplorationGoal&, double seconds) override {
    Require(entry.pending().has_value());
    const bool before_probe = arrival == EntryArrival::kBeforeProbe;
    Require(seconds == (before_probe ? 14. : 12.));
    // Pre-probe acknowledgement forgets room memory before cooling; a crawl
    // or driving crossing cools before recording/forgetting the entrance.
    Require(ledger.empty() == before_probe);
    trace.push_back("cool");
  }
  void QueueActiveRegionFinish() override {
    Require(!entry.pending() && !entry.locked() && ledger.empty());
    trace.push_back("queue");
  }
  void MarkRegionFinishPending() override {
    Require(arrival == EntryArrival::kDriving && trace.back() == "queue");
    trace.push_back("pending");
  }
  void SelectAfterEntry() override {
    Require(arrival != EntryArrival::kDriving && trace.back() == "queue");
    trace.push_back("select");
  }
  void ReportEntry(const EntryNotice& notice) override {
    Require(notice.kind == EntryNoticeKind::kEntered && notice.arrival == arrival);
    Require(notice.robot_x == 1.35 && notice.robot_y == .75);
    Require(!entry.pending());
    trace.push_back("report");
  }
};
}  // namespace
int main() {
  FrontierExplorer explorer;
  explorer.UpdateMap({30, 20, .1, 0., 0., std::vector<std::int8_t>(600, 0)});
  ExplorationGoal mouth;
  mouth.x = .35;
  mouth.y = .75;
  mouth.frontier_x = .8;
  mouth.frontier_y = .75;
  auto probe = mouth;
  probe.x = 1.35;
  for (auto arrival :
       {EntryArrival::kBeforeProbe, EntryArrival::kCrawl, EntryArrival::kDriving,
        EntryArrival::kNearProbe, EntryArrival::kNavigationSucceeded, EntryArrival::kSelection}) {
    EntryCoordinator entry;
    RoomVisitLedger ledger;
    entry.Lock(mouth, false, {600, 17});
    entry.CountAttempt();
    ledger.Remember(mouth, probe);
    Actions actions(entry, ledger, arrival);
    CompleteEntryArrival(entry, explorer, ledger, actions, arrival, 1.35, .75);
    Require(entry.attempts() == 0 && ledger.empty());
    Require(explorer.completed_goals() == 0 && explorer.failed_goals() == 0);
    if (arrival == EntryArrival::kDriving) {
      Require(actions.trace == std::vector<std::string>{"cool", "queue", "pending", "report"});
    } else if (arrival == EntryArrival::kBeforeProbe || arrival == EntryArrival::kCrawl) {
      Require(actions.trace == std::vector<std::string>{"cool", "queue", "select", "report"});
    } else {
      Require(actions.trace == std::vector<std::string>{"queue", "select", "report"});
    }
  }
}
