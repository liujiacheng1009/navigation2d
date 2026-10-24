#pragma once
#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/completion_execution.h"
#include "navigation2d/application/exploration/mission_lifecycle.h"
#include "navigation2d/application/exploration/tour_execution.h"
namespace navigation2d::test {
using exploration::CompletionActions;
using exploration::CompletionNotice;
using exploration::CompletionNoticeKind;
using exploration::MissionLifecycle;
using exploration::MissionPhase;
using exploration::TourExecution;
struct CompletionRecorder final : CompletionActions {
  MissionLifecycle& mission;
  TourExecution& tour;
  mutable std::vector<std::string> events;
  std::vector<CompletionNotice> notices;
  bool navigate = false, thrash = false, entry = false, pinch = false;
  bool rescue = false, waiting = false, seed_tour = false, partial = false;
  int lock_clears = 0;
  bool throw_navigation = false;
  CompletionRecorder(MissionLifecycle& mission, TourExecution& tour)
      : mission(mission), tour(tour) {}
  void ClearCompletionRouteCooling() override {
    assert(mission.phase() == MissionPhase::kCompleting && tour.goals().empty());
    events.push_back("clear_routes");
  }
  bool NavigateCompletionGoal(const ExplorationGoal&) override {
    events.push_back("navigate");
    if (throw_navigation) {
      throw std::runtime_error("injected navigation failure");
    }
    return navigate;
  }
  bool BreakMappedObstacleThrash() override {
    events.push_back("thrash");
    return thrash;
  }
  void LockEntryMouth(const ExplorationGoal&) override { events.push_back("lock"); }
  bool StartEntryProbe() override {
    events.push_back("entry");
    return entry;
  }
  void ClearEntryMouthLock() override {
    events.push_back("clear_lock");
    ++lock_clears;
  }
  bool TipPinchedNow() const override {
    events.push_back("pinch");
    return pinch;
  }
  bool TryRescueDistantMouths(const char*) override {
    events.push_back("rescue");
    return rescue;
  }
  bool ShouldKeepWaitingForFrontiers() const override {
    events.push_back("wait");
    return waiting;
  }
  void BeginReturnOrRescue(bool value) override {
    events.push_back("return");
    partial = value;
  }
  void CompletionStatus(const char* message) override { events.emplace_back(message); }
  void ReportCompletion(const CompletionNotice& notice) override {
    notices.push_back(notice);
    events.push_back("notice");
    if (notice.kind == CompletionNoticeKind::kEmpty) {
      assert(mission.phase() == MissionPhase::kCompleting);
    }
  }
  void BuildTour() override {
    assert(mission.phase() == MissionPhase::kExploring && tour.goals().empty());
    events.push_back("build");
    if (seed_tour) {
      tour.RetryLater(ExplorationGoal{.x = 2.});
    }
  }
  void StartCompletingProbe() override { events.push_back("probe_again"); }
  void SelectNextGoal() override { events.push_back("select"); }
};
inline ExplorationGrid CompletionMap() {
  ExplorationGrid map{50, 30, .1, 0., 0., std::vector<std::int8_t>(1500, 100)};
  for (int row = 4; row < 26; ++row) {
    for (int col = 4; col < 28; ++col) {
      map.cells[row * map.width + col] = 0;
    }
  }
  for (int row = 8; row < 22; ++row) {
    for (int col = 28; col < 45; ++col) {
      map.cells[row * map.width + col] = -1;
    }
  }
  return map;
}
}  // namespace navigation2d::test
