#include "navigation2d/application/exploration/entry_crawl_executor.h"

#include <cassert>
#include <string>
#include <vector>

#include "navigation2d/exploration/frontier_explorer.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
class Actions final : public EntryCrawlActions {
 public:
  explicit Actions(EntryCoordinator& entry) : entry_(entry) {}
  mutable std::vector<std::string> calls;
  bool blocked = false, probe_accepts = false;
  void AcknowledgeEntry(EntryArrival arrival) override {
    assert(arrival == EntryArrival::kCrawl);
    calls.emplace_back("arrival");
    entry_.Clear();
  }
  bool StartEntryProbe() override {
    assert(entry_.crawl_ticks() == 0);
    calls.emplace_back("probe");
    return probe_accepts;
  }
  void AbandonLockedEntryMouth(const std::string& reason) override {
    calls.emplace_back(reason.find("blocked") != std::string::npos ? "blocked" : "exhausted");
    entry_.Clear();
  }
  bool CrawlSegmentCutsWall(double, double, const ExplorationGoal&) const override {
    calls.emplace_back("segment");
    return blocked;
  }
  void PublishEntryCrawl(Twist2d command) override {
    assert(command.linear == .18 && command.angular == 0.);
    assert(entry_.crawl_ticks() > 1);
    calls.emplace_back("command");
  }

 private:
  EntryCoordinator& entry_;
};
}  // namespace
int main() {
  FrontierExplorer explorer;
  ExplorationGoal mouth{};
  mouth.frontier_x = 1.;
  auto target = mouth;
  target.x = 2.;
  EntryCoordinator entry;
  Actions actions(entry);
  auto reset = [&] {
    entry.Clear();
    entry.Lock(mouth, false, {});
    entry.StartCrawl();
    actions.calls.clear();
    actions.blocked = actions.probe_accepts = false;
  };
  assert(!HasActiveEntryCrawl(entry));
  reset();
  assert(HasActiveEntryCrawl(entry));
  ExecuteEntryCrawl(entry, explorer, target, {0., 0., 0.}, actions);
  assert((actions.calls == std::vector<std::string>{"segment", "command"}));
  reset();
  entry.CountAttempt();
  ExecuteEntryCrawl(entry, explorer, target, {1.6, 0., 0.}, actions);
  assert((actions.calls == std::vector<std::string>{"arrival"}));
  assert(!HasActiveEntryCrawl(entry));
  reset();
  ExecuteEntryCrawl(entry, explorer, target, {2., 0., 0.}, actions);
  assert((actions.calls == std::vector<std::string>{"blocked"}));
  reset();
  actions.blocked = true;
  ExecuteEntryCrawl(entry, explorer, target, {0., 0., 0.}, actions);
  assert((actions.calls == std::vector<std::string>{"segment", "blocked"}));
  // The last command is published before StopCrawl and the optional retry.
  for (int outcome = 0; outcome < 3; ++outcome) {
    reset();
    for (int tick = 0; tick < 39; ++tick) {
      entry.AdvanceCrawl({0., 0., 0., false}, target);
    }
    if (outcome == 0) {
      for (int attempt = 0; attempt < 6; ++attempt) {
        entry.CountAttempt();
      }
    }
    actions.probe_accepts = outcome == 1;
    ExecuteEntryCrawl(entry, explorer, target, {0., 0., 0.}, actions);
    const std::vector<std::string> expected =
        outcome == 0   ? std::vector<std::string>{"segment", "command", "exhausted"}
        : outcome == 1 ? std::vector<std::string>{"segment", "command", "probe"}
                       : std::vector<std::string>{"segment", "command", "probe", "exhausted"};
    assert(actions.calls == expected && entry.crawl_ticks() == 0);
  }
  reset();
  entry.StopCrawl();
  assert(!HasActiveEntryCrawl(entry));
  entry.StartCrawl();
  entry.ClearPending();
  assert(!HasActiveEntryCrawl(entry));
}
