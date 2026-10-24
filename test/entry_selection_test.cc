#include "navigation2d/application/exploration/entry_selection.h"

#include <cassert>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
class Actions final : public EntrySelectionActions {
 public:
  std::vector<std::string> calls;
  std::function<bool()> probe = [] { return false; };
  bool StartEntryProbe() override {
    calls.emplace_back("probe");
    return probe();
  }
  void AcknowledgeEntry(EntryArrival arrival) override {
    assert(arrival == EntryArrival::kSelection);
    calls.emplace_back("arrival");
  }
  void AbandonLockedEntryMouth(const std::string& reason) override {
    assert(reason == "entry blocked; leave mouth and continue tour (no reverse)");
    calls.emplace_back("abandon");
  }
  void DeferEntrySelection(EntrySelectionDelay delay) override {
    calls.emplace_back(delay == EntrySelectionDelay::kWideMouthCrawl ? "crawl" : "retry");
  }
};
ExplorationGoal Mouth() {
  ExplorationGoal goal{};
  goal.x = 0.;
  goal.y = 0.;
  goal.frontier_x = 1.;
  goal.frontier_y = 0.;
  return goal;
}
}  // namespace
int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  FrontierExplorer explorer;
  const auto mouth = Mouth();
  auto active = mouth;
  active.x = 2.;
  EntryCoordinator entry;
  Actions actions;
  auto select = [&] { return SelectLockedEntry(entry, explorer, std::nullopt, active, actions); };
  assert(select() == EntrySelectionResult::kContinueTour && actions.calls.empty());
  entry.Lock(mouth, false, {});
  entry.ClearPending();
  actions.probe = [] { return true; };
  assert(select() == EntrySelectionResult::kHandled);
  assert(entry.pending() && actions.calls == std::vector<std::string>{"probe"});
  actions.calls.clear();
  actions.probe = [&] {
    entry.CountAttempt();
    return false;
  };
  assert(select() == EntrySelectionResult::kHandled);
  assert((actions.calls == std::vector<std::string>{"probe", "retry"}));
  assert(entry.attempts() == 1 && entry.crawl_ticks() == 0);
  actions.calls.clear();
  actions.probe = [] { return false; };
  assert(select() == EntrySelectionResult::kContinueTour);
  assert((actions.calls == std::vector<std::string>{"probe", "abandon"}));
  entry.Lock(mouth, true, {});
  actions.calls.clear();
  assert(entry.EasyNow(explorer) && entry.AttemptBudget(explorer) == 14);
  assert(select() == EntrySelectionResult::kHandled && entry.crawl_ticks() == 1);
  assert((actions.calls == std::vector<std::string>{"probe", "crawl"}));
  entry.AdvanceCrawl({0., 0., 0., false}, active);
  const int crawl_ticks = entry.crawl_ticks();
  actions.calls.clear();
  assert(select() == EntrySelectionResult::kHandled && entry.crawl_ticks() == crawl_ticks);
  // A probe can exhaust its budget or clear the lock. Re-read state afterwards.
  entry.Lock(mouth, true, {});
  actions.calls.clear();
  actions.probe = [&] {
    for (int i = 0; i < 14; ++i) {
      entry.CountAttempt();
    }
    return false;
  };
  assert(select() == EntrySelectionResult::kContinueTour);
  assert((actions.calls == std::vector<std::string>{"probe", "abandon"}));
  actions.calls.clear();
  assert(select() == EntrySelectionResult::kContinueTour);
  assert(actions.calls == std::vector<std::string>{"abandon"});
  entry.Lock(mouth, true, {});
  actions.calls.clear();
  actions.probe = [&] {
    entry.Clear();
    return false;
  };
  assert(select() == EntrySelectionResult::kContinueTour);
  assert((actions.calls == std::vector<std::string>{"probe", "abandon"}));
  // An arrived pose bypasses plan and delay actions, even with a pending-only mouth.
  entry.Lock(mouth, false, {});
  entry.CountAttempt();
  actions.calls.clear();
  assert(SelectLockedEntry(entry, explorer, TourPose{1.6, 0., 0.}, active, actions) ==
         EntrySelectionResult::kHandled);
  assert(actions.calls == std::vector<std::string>{"arrival"});
  entry.Clear();
  entry.SetPending(mouth);
  assert(entry.AttemptBudget(explorer) == 6);
  actions.calls.clear();
  actions.probe = []() -> bool { throw std::runtime_error("navigation failed"); };
  bool threw = false;
  try {
    static_cast<void>(select());
  } catch (const std::runtime_error&) {
    threw = true;
  }
  assert(threw && actions.calls == std::vector<std::string>{"probe"});
  auto invalid = mouth;
  invalid.frontier_x = std::numeric_limits<double>::quiet_NaN();
  assert(!EntryCoordinator::Passable(invalid, explorer));
  entry.LockObserved(mouth, explorer, {17, 23});
  assert(!entry.easy() && entry.evidence().known_cells == 17 &&
         entry.evidence().map_revision == 23);
}
