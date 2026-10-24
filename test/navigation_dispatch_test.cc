#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation2d/application/exploration/dispatch_policy.h"
#include "navigation2d/application/exploration/navigation_session.h"
namespace {
using namespace navigation2d;
using namespace navigation2d::exploration;
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("dispatch contract failed");
  }
}
struct Actions final : DispatchActions {
  mutable std::vector<std::string> events;
  bool cuts = false, passable = false, accept_retry = false, prepared = true, occupied = false;
  bool expected_return = false;
  std::function<bool()> retry;
  void RejectPreparation(DispatchFailure failure) override {
    Require(!failure.reason.empty());
    occupied = failure.start_occupied;
    prepared = false;
    events.push_back("reject");
  }

  bool PreparedRouteCutsWall() const override {
    events.push_back("wall");
    return cuts;
  }
  void CoolRouteSignature(std::uint64_t signature) override {
    Require(signature == 42);
    events.push_back("cool");
  }
  void NoteMappedObstacleReject(const ExplorationGoal&) override { events.push_back("mapped"); }
  void ObservePreparedRoute(std::uint64_t signature, double) override {
    Require(signature == 42);
    events.push_back("observe");
  }
  bool PassableMouthGoal(const ExplorationGoal&) const override {
    events.push_back("passable");
    return passable;
  }
  bool RetryNavigation(const ExplorationGoal&, DispatchRetryRequest request) override {
    Require(!prepared && request.returning == expected_return);
    events.push_back(request.latest_map ? "latest" : "retry");
    return retry ? retry() : accept_retry;
  }

  void ReportDispatchRetry(DispatchRetryNotice) override { events.push_back("notice"); }
};
InitialPlanResult Probe() {
  InitialPlanResult result;
  result.state.path_signature = 42;
  result.state.global_planning.path_min_clearance_m = 10.;
  return result;
}
}  // namespace
int main() {
  DispatchPolicy policy;
  ExplorationGoal goal;
  auto probe = Probe();
  Actions actions;
  auto evaluate = [&](DispatchPurpose purpose = {false, false, false, false}) {
    return policy.Evaluate(goal, purpose, false, probe, actions);
  };
  Require(evaluate() == DispatchVerdict::kAcceptPrepared);
  Require(actions.events == std::vector<std::string>{"wall", "observe"});
  actions = {};
  actions.cuts = true;
  Require(evaluate() == DispatchVerdict::kRejected);
  Require(actions.events ==
          std::vector<std::string>{"wall", "reject", "cool", "latest", "cool", "mapped"});
  actions = {};
  actions.cuts = true;
  Require(evaluate({true, false, false, false}) == DispatchVerdict::kRejected);
  Require(actions.events == std::vector<std::string>{"wall", "reject", "latest", "cool"});

  actions = {};
  actions.cuts = true;
  actions.expected_return = true;
  Require(evaluate({false, true, false, false}) == DispatchVerdict::kRejected);
  Require(actions.events ==
          std::vector<std::string>{"wall", "reject", "cool", "latest", "cool", "mapped"});
  probe.state.path_signature = 0;
  actions = {};
  actions.cuts = true;
  Require(evaluate() == DispatchVerdict::kRejected);
  Require(actions.events == std::vector<std::string>{"wall", "reject", "latest", "mapped"});

  probe = Probe();
  probe.failure = InitialPlanFailure::kStartOccupied;
  probe.state.planning_failure_reason = "start occupied";
  actions = {};
  Require(evaluate() == DispatchVerdict::kRejected && actions.occupied);
  Require(actions.events == std::vector<std::string>{"reject"});
  probe.failure = InitialPlanFailure::kGoalOccupied;
  probe.state.planning_failure_reason = "goal occupied; no route";
  actions = {};
  Require(evaluate() == DispatchVerdict::kRejected && !actions.occupied);
  Require(actions.events == std::vector<std::string>{"reject"});
  probe.failure = InitialPlanFailure::kNoRoute;
  probe.state.planning_failure_reason.clear();
  actions = {};
  actions.accept_retry = true;
  Require(evaluate() == DispatchVerdict::kAcceptedRetry);
  Require(actions.events == std::vector<std::string>{"reject", "notice", "retry"});
  actions = {};
  actions.retry = [&] { return evaluate() == DispatchVerdict::kAcceptedRetry; };
  Require(evaluate() == DispatchVerdict::kRejected);
  Require(actions.events == std::vector<std::string>{"reject", "notice", "retry", "reject"});
  Require(!policy.active(DispatchPolicy::RetryKind::kPinch));

  probe = Probe();
  probe.state.global_planning.path_min_clearance_m = 0.;
  actions = {};
  actions.passable = true;
  Require(evaluate() == DispatchVerdict::kRejected);
  Require(actions.events == std::vector<std::string>{"wall", "observe", "cool", "reject", "retry",
                                                     "passable", "notice", "retry"});
  actions = {};
  Require(evaluate({false, false, false, true}) == DispatchVerdict::kRejected);
  Require(actions.events == std::vector<std::string>{"wall", "observe", "cool", "reject"});
  for (auto purpose : {DispatchPurpose{true, false, false, false},
                       {false, true, false, false},
                       {false, false, true, false}}) {
    actions = {};
    Require(evaluate(purpose) == DispatchVerdict::kAcceptPrepared);
    Require(actions.events == std::vector<std::string>{"wall", "observe"});
  }
  probe = Probe();
  probe.failure = InitialPlanFailure::kNoRoute;
  actions = {};
  actions.retry = []() -> bool { throw std::runtime_error("injected"); };
  try {
    static_cast<void>(evaluate());
    Require(false);
  } catch (const std::runtime_error& error) {
    Require(std::string(error.what()) == "injected");
  }
  Require(!policy.active(DispatchPolicy::RetryKind::kPinch));
}
