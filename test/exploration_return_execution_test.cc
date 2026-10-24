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
    calls.push_back(stage == Stage::kInitial ? "home" : "final_home");
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
}
