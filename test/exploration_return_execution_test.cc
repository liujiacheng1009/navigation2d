#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/return_coordinator.h"
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("return execution contract failed");
  }
}
class Actions : public navigation2d::exploration::ReturnActions {
 public:
  using Stage = navigation2d::exploration::ReturnPlanStage;
  navigation2d::exploration::ReturnCoordinator& coordinator;
  std::vector<std::string> calls;
  std::vector<bool> results;
  std::size_t next = 0;
  Actions(navigation2d::exploration::ReturnCoordinator& owner, std::vector<bool> accepted)
      : coordinator(owner), results(std::move(accepted)) {}
  bool PlanHome(Stage stage) override {
    calls.push_back(stage == Stage::kInitial
                        ? "home"
                        : (stage == Stage::kIdleFinal ? "idle_home" : "final_home"));
    return results.at(next++);
  }
  void HomeAccepted(Stage stage) override {
    Require(coordinator.escapes() == 0);  // Commit precedes the acceptance event.
    calls.push_back(stage == Stage::kInitial ? "home_accepted" : "final_accepted");
  }
  bool StartBreadcrumbTowardHome() override {
    calls.push_back("trail");
    return results.at(next++);
  }
  bool TryStartEscape() override {
    calls.push_back("escape");
    return results.at(next++);
  }
};
}  // namespace
int main() {
  using namespace navigation2d::exploration;
  ReturnCoordinator coordinator;
  coordinator.CountEscape();
  Actions direct(coordinator, {true});
  Require(coordinator.Execute(direct) == ReturnExecutionResult::kTracking);
  Require(direct.calls == std::vector<std::string>{"home", "home_accepted"});
  Actions escape(coordinator, {false, false, true});
  Require(coordinator.Execute(escape) == ReturnExecutionResult::kEscaping);
  Require(escape.calls == std::vector<std::string>{"home", "trail", "escape"});
  Require(coordinator.escapes() == 1);
  while (coordinator.escapes() < 5) {
    coordinator.CountEscape();
  }
  Actions capped(coordinator, {false, false, true});
  Require(coordinator.Execute(capped) == ReturnExecutionResult::kTracking);
  Require(capped.calls ==
          std::vector<std::string>{"home", "trail", "final_home", "final_accepted"});
  Require(coordinator.escapes() == 0);
  coordinator.CountEscape();
  coordinator.CountEscape();
  Actions final_trail(coordinator, {false, false, false, false, true});
  Require(coordinator.Execute(final_trail) == ReturnExecutionResult::kTracking);
  Require(final_trail.calls ==
          std::vector<std::string>{"home", "trail", "escape", "final_home", "trail"});
  Require(coordinator.escapes() == 2);
  Actions unavailable(coordinator, {false, false, false, false, false});
  Require(coordinator.Execute(unavailable) == ReturnExecutionResult::kUnavailable);

  ReturnCoordinator idle;
  Actions waiting(idle, {});
  Require(idle.ContinueIdle(true, waiting) == ReturnIdleResult::kWaiting);
  Require(waiting.calls.empty());
  idle.UseBreadcrumbs(true);
  Actions following(idle, {true});
  Require(idle.ContinueIdle(false, following) == ReturnIdleResult::kTracking);
  Require(following.calls == std::vector<std::string>{"trail"});
  idle.UseBreadcrumbs(false);
  Actions idle_escape(idle, {true});
  Require(idle.ContinueIdle(false, idle_escape) == ReturnIdleResult::kEscaping);
  Require(idle_escape.calls == std::vector<std::string>{"escape"});
  Require(idle.escapes() == 1);
  Actions after_failed_escape(idle, {false, true});
  Require(idle.ContinueIdle(false, after_failed_escape) == ReturnIdleResult::kTracking);
  Require(after_failed_escape.calls == std::vector<std::string>{"escape", "trail"});
  Require(idle.escapes() == 1);  // Idle trail acceptance does not reset the cap.
  Actions restart(idle, {false, false});
  Require(idle.ContinueIdle(false, restart) == ReturnIdleResult::kRestart);
  Require(!idle.partial());
  while (idle.escapes() < 5) {
    idle.CountEscape();
  }
  Actions idle_final(idle, {false, true});
  Require(idle.ContinueIdle(false, idle_final) == ReturnIdleResult::kTracking);
  Require(idle_final.calls == std::vector<std::string>{"trail", "idle_home"});
  Require(idle.escapes() == 5);
  Actions idle_stop(idle, {false, false});
  Require(idle.ContinueIdle(false, idle_stop) == ReturnIdleResult::kUnavailable);
  Require(!idle.partial());  // This exit preserves its original success verdict.
  Actions exhausted_trial(idle, {false});
  Require(idle.TryTrailOrEscape(exhausted_trial) == ReturnExecutionResult::kUnavailable);
  Require(exhausted_trial.calls == std::vector<std::string>{"trail"} && !idle.partial());
  ReturnCoordinator immobile;
  Actions immobile_actions(immobile, {false, false});
  Require(immobile.Recover(ReturnRecoveryMode::kImmobility, immobile_actions) ==
          ReturnExecutionResult::kUnavailable);
  Require(immobile.partial());
  Require(immobile_actions.calls == std::vector<std::string>{"trail", "escape"});
  ReturnCoordinator cut;
  cut.CountEscape();
  Actions cut_actions(cut, {false, false, true});
  Require(cut.Recover(ReturnRecoveryMode::kPathFailure, cut_actions) ==
          ReturnExecutionResult::kTracking);
  Require(cut_actions.calls == std::vector<std::string>{"trail", "escape", "final_home"});
  Require(cut.escapes() == 1 && !cut.partial());
  Actions cut_unavailable(cut, {false, false, false});
  Require(cut.Recover(ReturnRecoveryMode::kPathFailure, cut_unavailable) ==
          ReturnExecutionResult::kUnavailable);
  Require(cut.partial());
}
