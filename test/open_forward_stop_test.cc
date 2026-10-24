#include "navigation2d/application/exploration/open_forward_stop.h"

#include <cassert>
#include <stdexcept>

#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/test/entry_recovery_fixture.h"
namespace {
struct Clock final : navigation2d::exploration::GoalExecutionClock {
  std::int64_t now_ns = 1000000000LL;
  mutable int reads = 0;
  std::int64_t ExecutionNowNs() const override {
    ++reads;
    return now_ns;
  }
};
}  // namespace
int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  FrontierExplorer explorer;
  EntryCoordinator entry;
  GoalRecoverySession recovery;
  GoalExecutionWatch watch;
  Clock clock;
  test::EntryRecoverySpy actions(entry, recovery);
  const auto execute = [&] {
    ExecuteOpenForwardStop(entry, explorer, recovery, watch, clock, {.2, .1}, actions);
  };
  // A locked mouth is restored before the lazy time check, without querying
  // open-space geometry or consuming a probe when its progress stamp is zero.
  entry.Lock(test::RecoveryMouth(), false, {});
  entry.ClearPending();
  execute();
  assert(entry.pending() && actions.calls.empty() && clock.reads == 0);
  watch.Start(1000000000LL, 1000000000LL);
  clock.now_ns = 5000000000LL;
  execute();
  assert(actions.calls.empty());
  ++clock.now_ns;
  actions.probe_accepts = true;
  execute();
  assert((actions.calls ==
          std::vector<std::string>{"cancel", "evidence", "probe", "status:COMPLETING"}));
  entry.Clear();
  actions.ResetTrace();
  actions.scrape = true;
  clock.reads = 0;
  execute();
  assert((actions.calls == std::vector<std::string>{"geometry", "peel"}) && clock.reads == 0);
  actions.ResetTrace();
  clock.now_ns = 7000000000LL;
  execute();
  assert(actions.calls == std::vector<std::string>{"geometry"});
  actions.ResetTrace();
  ++clock.now_ns;
  actions.replan_accepts = true;
  execute();
  assert((actions.calls == std::vector<std::string>{"geometry", "replan"}) &&
         recovery.open_replans == 1);
  actions.ResetTrace();
  actions.replan_accepts = true;
  execute();
  assert((actions.calls == std::vector<std::string>{"geometry", "defer", "cancel"}));
  assert(actions.deferred_s == .35 && recovery.open_replans == 1);
  actions.ResetTrace();
  recovery.open_replans = 0;
  execute();
  assert((actions.calls == std::vector<std::string>{"geometry", "replan", "defer", "cancel"}));
  assert(recovery.open_replans == 0);
  actions.ResetTrace();
  actions.on_replan = [] { throw std::runtime_error("replan failed"); };
  bool threw = false;
  try {
    execute();
  } catch (const std::runtime_error&) {
    threw = true;
  }
  assert(threw && (actions.calls == std::vector<std::string>{"geometry", "replan"}));
}
