#pragma once
#include <cassert>
#include <functional>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/exploration_mission.h"
#include "navigation2d/application/exploration/goal_failure_execution.h"
#include "navigation2d/application/exploration/return_coordinator.h"
#include "navigation2d/exploration/frontier_explorer.h"
namespace navigation2d::test {
using namespace exploration;
struct FailureScene : GoalFailureActions, GoalExecutionClock {
  ExplorationMission mission;
  ReturnCoordinator returning;
  EntryCoordinator entry;
  TourExecution tour;
  GoalRecoverySession recovery;
  RetreatExecution retreat;
  FrontierExplorer explorer;
  GoalExecutionWatch watch;
  GoalFailureExecution execution{mission,  returning, entry,    tour,
                                 recovery, retreat,   explorer, watch};
  NavigationState state;
  ExplorationGoal goal;
  mutable std::vector<std::string> events;
  std::string message;
  std::int64_t now_ns = 2000000000LL;
  bool corner = false, pinch = false, pose = true, force_return = false;
  bool fallback = false, replan = true, escape = false, breadcrumbs = false, probe = true;
  std::function<void()> on_replan;
  FailureScene() {
    explorer.UpdateMap({30, 30, .1, 0., 0., std::vector<std::int8_t>(900, 0)});
    goal.x = goal.y = goal.frontier_x = goal.frontier_y = 1.;
    tour.RetryLater(goal);
    tour.RetryLater(goal);
    watch.Start(1000000000LL, 1000000000LL);
    state.status = NavigationStatus::kBlocked;
  }
  void Run(double front = 1., double side = 1.) {
    execution.Resolve(state, front, side, *this, *this);
  }
  std::int64_t ExecutionNowNs() const override {
    events.emplace_back("clock");
    return now_ns;
  }
  const ExplorationGoal& ActiveGoal() const override { return goal; }
  double ActivePathMargin(double margin) const override { return margin; }
  bool IsCornerTipStall(double, double, double) const override {
    events.emplace_back("corner");
    return corner;
  }
  bool IsPinchGeometryStall(double, double, double) const override {
    events.emplace_back("pinch");
    return pinch;
  }
  void ReportExecutionFailure(const NavigationState&) override { events.emplace_back("report"); }
  bool NoteFirstSegmentFailure(const NavigationState&) override {
    events.emplace_back("first");
    return force_return;
  }
  bool TryReturnSegmentFallback() override {
    events.emplace_back("fallback");
    return fallback;
  }
  bool FailurePoseReady() const override {
    events.emplace_back("pose");
    return pose;
  }
  void PrepareNoMotionEntry(const NavigationState&) override { events.emplace_back("crawl"); }
  void DemoteActiveGoal(const char* reason) override {
    events.emplace_back("demote-active");
    message = reason;
  }
  void BeginReturn(bool partial) override {
    assert(partial);
    events.emplace_back("return");
    mission.BeginReturning();
  }
  bool StartBreadcrumbTowardHome() override {
    events.emplace_back("breadcrumbs");
    return breadcrumbs;
  }
  void FinishCompleting() override { events.emplace_back("completion"); }
  void Finish(bool success, const std::string& reason) override {
    assert(success && returning.partial());
    events.emplace_back("finish");
    message = reason;
  }
  void CancelRecoveryNavigation() override { events.emplace_back("cancel"); }
  void SelectAfterEntry() override { events.emplace_back("select"); }
  void PublishStatus(const std::string& phase, const std::string& reason) override {
    events.push_back("status:" + phase);
    message = reason;
  }
  void AbandonLockedEntryMouth(const std::string&) override {
    events.emplace_back("abandon");
    entry.Clear();
  }
  bool StartEntryProbe() override {
    events.emplace_back("probe");
    entry.CountAttempt();
    return probe;
  }
  void PrepareDoorjambRetry() override { events.emplace_back("evidence"); }
  bool OpenForwardWallScrape(double, double) const override {
    events.emplace_back("open-geometry");
    return false;
  }
  bool BreakWallScrapeThrash(const char* reason) override {
    events.emplace_back("scrape");
    message = reason;
    return false;
  }
  bool ReplanActiveGoalOnLatestMap(const char* reason) override {
    events.emplace_back("replan");
    message = reason;
    if (on_replan) {
      on_replan();
    }
    return replan;
  }
  void DeferViewpointSelection(double, const char*) override { events.emplace_back("defer"); }
  bool EscapeOrShortRetreat(const char*, const char*) override {
    events.emplace_back("peel");
    return escape;
  }
  bool TryStartEscape() override {
    events.emplace_back("escape");
    return escape;
  }
  void DemoteViewpointForSeconds(const ExplorationGoal&, double seconds) override {
    assert(seconds == 8. || seconds == 25.);
    events.emplace_back("demote");
  }
};
}  // namespace navigation2d::test
