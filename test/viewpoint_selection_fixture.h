#pragma once
#include <cassert>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/viewpoint_selection_actions.h"
#include "navigation2d/test/selection_exhaustion_fixture.h"
namespace navigation2d::test {
class ViewpointSelectionRecorder final : public exploration::ViewpointSelectionActions {
 public:
  SelectionRecorder exhausted;
  std::vector<ExplorationGoal> planned;
  std::string reason = "no route";
  bool accept = false, occupied = false, escape = false;
  int builds = 0, accepted = 0, prepares = 0, escapes = 0, cooled = 0, rejected = 0;
  std::optional<exploration::TourPose> rescue_robot = exploration::TourPose{0., 1.5, 0.};
  std::vector<std::string> rescue_events;
  bool rescue_pinched = false, rescue_navigation = false;
  std::function<void()> on_build, on_select;
  std::optional<exploration::TourPose> TourRobot() const override { return rescue_robot; }
  void ReportSelectionRescue(const char*, bool missing) override {
    rescue_events.emplace_back(missing ? "missing" : "report");
  }
  void DiscardRescueNavigation() override { rescue_events.emplace_back("discard"); }
  void DeferRescueRecovery(const char*) override { rescue_events.emplace_back("defer-rescue"); }
  void SelectNextGoal() override {
    rescue_events.emplace_back("select");
    if (on_select) {
      on_select();
    }
  }
  bool HasRescueNavigation() const override { return rescue_navigation; }
  void BuildTour() override {
    ++builds;
    rescue_events.emplace_back("build");
    if (on_build) {
      on_build();
    }
  }
  bool NavigateViewpoint(const ExplorationGoal& goal) override {
    planned.push_back(goal);
    return accept;
  }
  exploration::ViewpointPlanFailure LastViewpointFailure() const override {
    return {occupied, reason, 42};
  }
  void ViewpointAccepted() override { ++accepted; }
  void PreparePinchedViewpoint(const std::string& value) override {
    assert(value == reason);
    ++prepares;
  }
  bool EscapeOrShortRetreat(const char*, const char*) override {
    ++escapes;
    rescue_events.emplace_back("peel");
    return escape;
  }
  void ClearSelectionRouteCooling() override {
    ++cooled;
    rescue_events.emplace_back("cooling");
  }
  void LockEntryMouth(const ExplorationGoal&) override { assert(false); }
  void DeferViewpointRetry() override { assert(false); }
  void ReportSelectionSkip(exploration::SelectionSkip, const ExplorationGoal&) override {}
  bool RecentlyDemoted(const ExplorationGoal&) override { return false; }
  bool GoalIsBehindTrail(const ExplorationGoal&) const override { return false; }
  bool StartEntryProbe() override {
    assert(false);
    return false;
  }
  void AcknowledgeEntry(exploration::EntryArrival) override { assert(false); }
  void AbandonLockedEntryMouth(const std::string&) override { assert(false); }
  void DeferEntrySelection(exploration::EntrySelectionDelay) override { assert(false); }
  void CoolRejectedSignature(std::uint64_t) override { ++cooled; }
  std::int64_t RejectionDeadlineNs(double) const override { return 10000000000LL; }
  void ReportViewpointRejection(const exploration::ViewpointRejectionNotice&) override {
    ++rejected;
  }
  void StopSelectionMotion() override { exhausted.StopSelectionMotion(); }
  void NoteKnownProgress(std::size_t known) override { exhausted.NoteKnownProgress(known); }
  void QueueActiveRegionFinish() override { exhausted.QueueActiveRegionFinish(); }
  void DeferBlockedSelection(const char* value) override { exhausted.DeferBlockedSelection(value); }
  void SelectionStatus(const char* value) override { exhausted.SelectionStatus(value); }
  void ClearBlockedSelection() override { exhausted.ClearBlockedSelection(); }
  bool SelectionFrontSwingClear() const override { return exhausted.SelectionFrontSwingClear(); }
  bool TipPinchedNow() const override { return rescue_pinched || exhausted.TipPinchedNow(); }
  bool TryRescueDistantMouths(const char* reason) override {
    return exhausted.TryRescueDistantMouths(reason);
  }
  bool ShouldKeepWaitingForFrontiers() const override {
    return exhausted.ShouldKeepWaitingForFrontiers();
  }
  void StartCompletingProbe() override { exhausted.StartCompletingProbe(); }
  void BeginReturnOrRescue(bool value) override { exhausted.BeginReturnOrRescue(value); }
  void BeginReturn(bool value) override { exhausted.BeginReturn(value); }
};
}  // namespace navigation2d::test
