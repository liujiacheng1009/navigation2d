#pragma once
#include <cassert>
#include <functional>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/open_forward_stop.h"
namespace navigation2d::test {
using namespace exploration;
struct EntryRecoverySpy final : OpenForwardStopActions {
  EntryCoordinator& entry;
  GoalRecoverySession& recovery;
  mutable std::vector<std::string> calls;
  std::function<void()> on_probe, on_abandon, on_select, on_replan;
  bool probe_accepts = false, scrape = false, replan_accepts = false;
  std::string message;
  double deferred_s = -1.;
  EntryRecoverySpy(EntryCoordinator& e, GoalRecoverySession& r) : entry(e), recovery(r) {}
  bool StartEntryProbe() override {
    calls.emplace_back("probe");
    if (on_probe) {
      on_probe();
    } else {
      entry.CountAttempt();
    }
    return probe_accepts;
  }
  void AbandonLockedEntryMouth(const std::string& reason) override {
    calls.emplace_back("abandon");
    message = reason;
    if (on_abandon) {
      on_abandon();
    }
    entry.Clear();
  }
  void PublishStatus(const std::string& phase, const std::string& reason) override {
    calls.emplace_back("status:" + phase);
    message = reason;
  }
  void SelectAfterEntry() override {
    calls.emplace_back("select");
    if (on_select) {
      on_select();
    }
  }
  void CancelRecoveryNavigation() override { calls.emplace_back("cancel"); }
  void PrepareDoorjambRetry() override { calls.emplace_back("evidence"); }
  bool OpenForwardWallScrape(double clearance, double side) const override {
    assert(clearance == .2 && side == .1);
    calls.emplace_back("geometry");
    return scrape;
  }
  bool BreakWallScrapeThrash(const char* reason) override {
    calls.emplace_back("peel");
    message = reason;
    return false;
  }
  bool ReplanActiveGoalOnLatestMap(const char* reason) override {
    calls.emplace_back("replan");
    message = reason;
    if (on_replan) {
      on_replan();
    }
    return replan_accepts;
  }
  void DeferViewpointSelection(double seconds, const char* reason) override {
    calls.emplace_back("defer");
    deferred_s = seconds;
    message = reason;
  }
  void ResetTrace() {
    calls.clear();
    message.clear();
    on_probe = nullptr;
    on_abandon = nullptr;
    on_select = nullptr;
    on_replan = nullptr;
    probe_accepts = scrape = replan_accepts = false;
    deferred_s = -1.;
  }
};
inline ExplorationGoal RecoveryMouth() {
  ExplorationGoal mouth;
  mouth.frontier_x = 1.;
  return mouth;
}
}  // namespace navigation2d::test
