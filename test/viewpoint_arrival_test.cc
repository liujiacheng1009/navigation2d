#include "navigation2d/application/exploration/viewpoint_arrival.h"

#include <cassert>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/entry_coordinator.h"
#include "navigation2d/application/exploration/tour_execution.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
class Actions final : public ViewpointArrivalActions {
 public:
  explicit Actions(EntryCoordinator& entry) : entry_(entry) {}
  std::vector<std::string> calls;
  bool accepts = false;
  void LockEntryMouth(const ExplorationGoal& goal) override {
    calls.emplace_back("lock");
    entry_.Lock(goal, false, {});
  }
  bool StartEntryProbe() override {
    calls.emplace_back("probe");
    entry_.CountAttempt();
    return accepts;
  }
  void AcknowledgeEntry(EntryArrival arrival) override {
    calls.emplace_back(arrival == EntryArrival::kNearProbe ? "near-entry" : "succeeded-entry");
    entry_.Clear();
  }
  void AbandonLockedEntryMouth(const std::string&) override {
    calls.emplace_back("abandon");
    entry_.Clear();
  }
  void ClearEntryMouthLock() override {
    calls.emplace_back("clear");
    entry_.Clear();
  }
  void QueueActiveRegionFinish() override { calls.emplace_back("queue"); }
  void DeferViewpointSelection(double seconds, const char* message) override {
    if (seconds == .35) {
      assert(std::string(message) == "at doorway; retrying centered entry probe");
      calls.emplace_back("retry");
    } else {
      assert(seconds == 0.);
      calls.emplace_back(message);
    }
  }
  void ReportViewpointArrival(ViewpointArrivalNotice kind, const ExplorationGoal&, int attempts,
                              double depth) override {
    assert(kind == ViewpointArrivalNotice::kEntryIncomplete && attempts == 6);
    assert(depth == -1.);
    calls.emplace_back("incomplete");
  }

 private:
  EntryCoordinator& entry_;
};
}  // namespace
int main() {
  FrontierExplorer explorer;
  EntryCoordinator entry;
  TourExecution tour;
  ViewpointArrival arrival;
  Actions actions(entry);
  ExplorationGoal mouth{};
  mouth.frontier_x = 1.;
  auto target = mouth;
  target.x = 2.;
  auto lock = [&] {
    entry.Lock(mouth, false, {});
    actions.calls.clear();
  };
  lock();
  arrival.Near(explorer, entry, tour, target, {0., 0., 0.}, actions);
  assert((actions.calls == std::vector<std::string>{"probe", "retry"}));
  lock();
  for (int i = 0; i < 5; ++i) {
    entry.CountAttempt();
  }
  arrival.Near(explorer, entry, tour, target, {0., 0., 0.}, actions);
  assert((actions.calls == std::vector<std::string>{"probe", "abandon"}));
  lock();
  for (int i = 0; i < 6; ++i) {
    entry.CountAttempt();
  }
  arrival.Near(explorer, entry, tour, target, {0., 0., 0.}, actions);
  assert(actions.calls == std::vector<std::string>{"abandon"});
  lock();
  entry.CountAttempt();
  arrival.Near(explorer, entry, tour, target, {1.6, 0., 0.}, actions);
  assert(actions.calls == std::vector<std::string>{"near-entry"});
  lock();
  entry.CountAttempt();
  arrival.Succeeded(explorer, entry, tour, target, {1.6, 0., 0.}, actions);
  assert(actions.calls == std::vector<std::string>{"succeeded-entry"});
  lock();
  actions.accepts = true;
  arrival.Succeeded(explorer, entry, tour, target, {0., 0., 0.}, actions);
  assert(actions.calls == std::vector<std::string>{"probe"});
  actions.accepts = false;
  lock();
  for (int i = 0; i < 5; ++i) {
    entry.CountAttempt();
  }
  arrival.Succeeded(explorer, entry, tour, target, {0., 0., 0.}, actions);
  assert((actions.calls == std::vector<std::string>{"probe", "incomplete", "abandon"}));
  actions.calls.clear();
  tour.RetryLater(target);
  arrival.Near(explorer, entry, tour, target, {0., 0., 0.}, actions);
  assert(tour.goals().empty());
  assert((actions.calls ==
          std::vector<std::string>{"clear", "arrived at viewpoint; continuing tour"}));
  // Mid-drive entry creates one region-finish obligation, consumed at success.
  actions.calls.clear();
  arrival.MarkRegionFinishPending();
  tour.RetryLater(target);
  arrival.Succeeded(explorer, entry, tour, target, {0., 0., 0.}, actions);
  assert(tour.goals().empty());
  assert((actions.calls ==
          std::vector<std::string>{"queue", "active room unfinished; covering local leftovers"}));
  actions.calls.clear();
  arrival.Succeeded(explorer, entry, tour, target, {0., 0., 0.}, actions);
  assert(actions.calls ==
         std::vector<std::string>{"frontier reached; continuing active region tour"});
}
