#pragma once
#include <cassert>
#include <cstdint>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/escape_runtime.h"
#include "navigation2d/application/exploration/motion_output.h"
namespace navigation2d::test {
class EscapeRecorder final : public exploration::EscapeRuntime {
 public:
  exploration::EscapeExecution& owner;
  exploration::GoalRecoverySession& recovery;
  mutable std::vector<std::string> events;
  exploration::BodyRanges body{2., 2., 2., 2., 2.};
  exploration::EscapeExitContext context{exploration::MissionPhase::kExploring, false, false};
  bool ready = true, occupied = false, returning = false, break_peel = false, navigate = false;
  std::int64_t now_ns = 1000000000;
  mutable std::int64_t deadline = 0;
  Twist2d started;
  EscapeRecorder(exploration::EscapeExecution& value, exploration::GoalRecoverySession& budget)
      : owner(value), recovery(budget) {}
  bool EscapeInputsReady() const override {
    events.emplace_back("ready");
    return ready;
  }
  std::int64_t ExecutionNowNs() const override {
    events.emplace_back("now");
    return now_ns;
  }
  exploration::BodyRanges EscapeBodyRanges() const override {
    events.emplace_back("body");
    return body;
  }
  bool EscapeStartOccupied() const override {
    events.emplace_back("occupied");
    return occupied;
  }
  bool EscapeReturning() const override {
    events.emplace_back("returning");
    return returning;
  }
  std::int64_t EscapeDeadlineNs(double hold) const override {
    events.emplace_back("deadline");
    deadline = now_ns + static_cast<std::int64_t>(hold * 1e9);
    return deadline;
  }
  void ReportEscapeSkipped(const exploration::BodyRanges&) override {
    events.emplace_back("skipped");
  }
  void ReportEscapeStarted(const exploration::BodyRanges&, Twist2d command, double) override {
    started = command;
    assert(owner.scheduled());
    events.emplace_back("started");
  }
  void EscapeDrivingStatus() override { events.emplace_back("driving"); }
  exploration::EscapeExitContext EscapeExitState() const override {
    assert(!owner.scheduled());
    events.emplace_back("context");
    return context;
  }
  void BeginReturn(bool partial) override {
    assert(partial && !owner.resume_return() && recovery.resume_after_escape);
    events.emplace_back("return");
  }
  void ResumeEntryAfterEscape() override { events.emplace_back("entry"); }
  bool BreakWallScrapeAfterPeel() override {
    events.emplace_back("break");
    return break_peel;
  }
  bool NavigateAfterEscape() override {
    events.emplace_back("navigate");
    return navigate;
  }
  void EscapeGoalResumed() override { events.emplace_back("resumed"); }
  void DemoteActiveGoal(const char*) override { events.emplace_back("demote"); }
  void FinishCompleting() override { events.emplace_back("complete"); }
  void SelectAfterEscape() override { events.emplace_back("select"); }
};
class EscapeMotion final : public exploration::MotionOutput {
 public:
  exploration::EscapeExecution& owner;
  std::vector<std::string>& events;
  std::vector<Twist2d> commands;
  bool expired = false;
  EscapeMotion(exploration::EscapeExecution& value, std::vector<std::string>& log)
      : owner(value), events(log) {}
  void Send(Twist2d command) override {
    if (expired) {
      assert(!owner.scheduled() && command.linear == 0. && command.angular == 0.);
    }
    commands.push_back(command);
    events.emplace_back("motion");
  }
};
}  // namespace navigation2d::test
