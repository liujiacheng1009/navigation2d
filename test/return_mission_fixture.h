#pragma once
#include <cassert>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/return_mission_actions.h"
namespace navigation2d::test {
class ReturnMissionRecorder final : public exploration::ReturnMissionActions {
 public:
  exploration::ReturnCoordinator& owner;
  mutable std::vector<std::string> events;
  bool visit = false, coverage = true, plan = true, breadcrumb = false, escape = false;
  bool navigate = true, finished = false, success = false, restart_partial = false;
  ExplorationGoal planned;
  std::string message;
  explicit ReturnMissionRecorder(exploration::ReturnCoordinator& value) : owner(value) {}
  bool TryVisitUnenteredSpace() override {
    events.emplace_back("visit");
    return visit;
  }
  void CommitReturnPhase() override { events.emplace_back("phase"); }
  void SuspendExplorationForReturn() override {
    assert(owner.returning());
    events.emplace_back("suspend");
  }
  bool ReturnCoverageComplete() const override {
    events.emplace_back("coverage");
    return coverage;
  }
  void ReportReturnStart(exploration::ReturnStartNotice notice) override {
    const bool partial = notice == exploration::ReturnStartNotice::kForcedPartial;
    if (partial) {
      assert(owner.partial());
    }
    events.emplace_back(partial ? "partial" : "incomplete");
  }
  void ClearReturnRouteCooling() override { events.emplace_back("cool"); }
  void PrepareReturnEvidence(bool replay) override {
    events.emplace_back(replay ? "evidence-start" : "evidence");
  }
  bool NavigateReturnGoal(const ExplorationGoal& goal) override {
    planned = goal;
    events.emplace_back("navigate");
    if (navigate) {
      owner.AcceptNavigationMode(true);
    }
    return navigate;
  }
  void ReportBreadcrumbHop(const ExplorationGoal& goal, double, double, double distance) override {
    assert(owner.breadcrumbs() && owner.returning() && goal.x == planned.x && distance > .4);
    events.emplace_back("hop");
  }
  void ResumeReturnEscape(bool replay) override {
    events.emplace_back(replay ? "resume-start" : "resume");
  }
  void Finish(bool value, const std::string& text) override {
    finished = true;
    success = value;
    message = text;
    events.emplace_back("finish");
  }
  void BeginReturn(bool partial) override {
    restart_partial = partial;
    events.emplace_back("restart");
  }
  bool PlanHome(exploration::ReturnPlanStage stage) override {
    events.emplace_back(stage == exploration::ReturnPlanStage::kInitial ? "home" : "final-home");
    return plan;
  }
  void HomeAccepted(exploration::ReturnPlanStage) override { events.emplace_back("accepted"); }
  bool StartBreadcrumbTowardHome() override {
    events.emplace_back("trail");
    return breadcrumb;
  }
  bool TryStartEscape() override {
    events.emplace_back("escape");
    return escape;
  }
  bool PlanDirectAfterHop(const ExplorationGoal&) override {
    assert(false);
    return false;
  }
  void DirectHomeAfterHopAccepted() override { assert(false); }
  void FinishReturn(bool) override { assert(false); }
};
}  // namespace navigation2d::test
