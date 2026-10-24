#pragma once
#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/selection_exhaustion.h"
namespace navigation2d::test {
class SelectionRecorder final : public exploration::SelectionExhaustionActions {
 public:
  mutable std::vector<std::string> events;
  bool pinched = false, swing_clear = false, rescue = false, wait = false, throw_stop = false;
  bool partial = false;
  int returns = 0, direct_returns = 0, probes = 0;
  std::size_t known = 0;
  void StopSelectionMotion() override {
    events.emplace_back("stop");
    if (throw_stop) {
      throw std::runtime_error("publisher failed");
    }
  }
  void NoteKnownProgress(std::size_t value) override {
    known = value;
    events.emplace_back("known");
  }
  void QueueActiveRegionFinish() override { events.emplace_back("queue"); }
  void DeferBlockedSelection(const char*) override { events.emplace_back("defer"); }
  void SelectionStatus(const char* message) override { events.emplace_back(message); }
  void ClearBlockedSelection() override { events.emplace_back("clear"); }
  bool SelectionFrontSwingClear() const override {
    events.emplace_back("swing");
    return swing_clear;
  }
  bool TipPinchedNow() const override {
    events.emplace_back("pinch");
    return pinched;
  }
  bool TryRescueDistantMouths(const char*) override {
    events.emplace_back("rescue");
    return rescue;
  }
  bool ShouldKeepWaitingForFrontiers() const override {
    events.emplace_back("wait");
    return wait;
  }
  void StartCompletingProbe() override {
    ++probes;
    events.emplace_back("probe");
  }
  void BeginReturnOrRescue(bool value) override {
    partial = value;
    ++returns;
    events.emplace_back("return");
  }
  void BeginReturn(bool value) override {
    assert(!value);
    ++direct_returns;
    events.emplace_back("direct_return");
  }
};
}  // namespace navigation2d::test
