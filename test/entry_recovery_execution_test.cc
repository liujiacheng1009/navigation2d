#include "navigation2d/application/exploration/entry_recovery_execution.h"

#include <cassert>
#include <stdexcept>

#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/test/entry_recovery_fixture.h"
int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration;
  FrontierExplorer explorer;
  EntryCoordinator entry;
  GoalRecoverySession recovery;
  test::EntryRecoverySpy actions(entry, recovery);
  const auto mouth = test::RecoveryMouth();
  const auto reset = [&](bool easy = false) {
    entry.Clear();
    entry.Lock(mouth, easy, {});
    actions.ResetTrace();
  };
  assert(!ResumeEntryAfterRetreat(entry, explorer, actions) && actions.calls.empty());
  entry.SetPending(mouth);
  assert(!ResumeEntryAfterRetreat(entry, explorer, actions) && actions.calls.empty());
  reset();
  entry.ClearPending();
  actions.probe_accepts = true;
  ResumeLockedEntryAfterPeel(entry, explorer, actions);
  assert(entry.pending() && entry.attempts() == 1);
  assert((actions.calls == std::vector<std::string>{"probe", "status:COMPLETING"}));
  assert(actions.message == "resumed locked mouth after peel");
  reset(true);
  entry.StartCrawl();
  auto crawl_target = mouth;
  crawl_target.x = 2.;
  entry.AdvanceCrawl({0., 0., 0., false}, crawl_target);
  const auto crawl = entry.crawl_ticks();
  ResumeLockedEntryAfterPeel(entry, explorer, actions);
  assert(entry.crawl_ticks() == crawl);
  assert((actions.calls == std::vector<std::string>{"probe", "status:COMPLETING"}));
  assert(actions.message == "peel done; wide mouth crawl instead of abandon");
  reset(true);
  for (int i = 0; i < 14; ++i) {
    entry.CountAttempt();
  }
  ResumeLockedEntryAfterPeel(entry, explorer, actions);
  assert(entry.crawl_ticks() == 1 &&
         actions.calls == std::vector<std::string>{"status:COMPLETING"});
  reset();
  entry.ClearPending();
  for (int i = 0; i < 6; ++i) {
    entry.CountAttempt();
  }
  recovery.Begin(0., 0.);
  recovery.open_replans = 2;
  actions.on_abandon = [&] { assert(entry.pending() && recovery.open_replans == 2); };
  actions.on_select = [&] { assert(recovery.active && recovery.open_replans == 2); };
  ResumeLockedEntryAfterPeel(entry, explorer, actions);
  assert((actions.calls == std::vector<std::string>{"abandon", "select"}));
  reset();
  actions.probe_accepts = true;
  RetryLockedEntryAfterDoorjamb(entry, explorer, actions);
  assert((actions.calls ==
          std::vector<std::string>{"cancel", "evidence", "probe", "status:COMPLETING"}));
  reset();
  for (int i = 0; i < 6; ++i) {
    entry.CountAttempt();
  }
  RetryLockedEntryAfterDoorjamb(entry, explorer, actions);
  assert((actions.calls == std::vector<std::string>{"cancel", "evidence", "abandon", "select"}));
  // Retreat must restore the lock even when another pending goal is present.
  reset();
  auto other = mouth;
  other.frontier_x = 8.;
  entry.SetPending(other);
  actions.on_probe = [&] { assert(entry.pending()->frontier_x == 1.); };
  assert(!ResumeEntryAfterRetreat(entry, explorer, actions));
  assert(actions.calls == std::vector<std::string>{"probe"});
  reset();
  actions.probe_accepts = true;
  ResolveLockedEntryStall(entry, explorer, recovery, actions);
  assert(actions.calls == std::vector<std::string>{"probe"} && recovery.active);
  reset();
  actions.on_abandon = [&] { assert(recovery.active && recovery.open_replans == 2); };
  actions.on_select = [&] { assert(!recovery.active && recovery.open_replans == 0); };
  ResolveLockedEntryStall(entry, explorer, recovery, actions);
  assert((actions.calls == std::vector<std::string>{"probe", "abandon", "select"}));
  reset();
  actions.on_probe = [] { throw std::runtime_error("probe failed"); };
  bool threw = false;
  try {
    ResolveLockedEntryStall(entry, explorer, recovery, actions);
  } catch (const std::runtime_error&) {
    threw = true;
  }
  assert(threw && actions.calls == std::vector<std::string>{"probe"} && entry.pending());
}
