#include <cassert>
#include <chrono>
#include <functional>
#include <limits>
#include <stdexcept>

#include "navigation2d/control/local_controller.h"
#include "navigation2d/test/algorithm_contract_fixture.h"
namespace {
using namespace navigation2d;
class LegacyController final : public LocalController {
 public:
  mutable int calls = 0;
  bool deliberate = false;
  std::function<Twist2d()> run;
  std::function<void()> diagnostic_action;
  Twist2d Compute(const Path&, const Pose2d&, Twist2d, const LayeredCostmap&,
                  const std::vector<PredictedObstacle>&) const override {
    ++calls;
    return run();
  }
  ControllerDiagnostics Diagnostics() const override {
    if (diagnostic_action) {
      diagnostic_action();
    }
    ControllerDiagnostics d;
    d.intentional_stop = deliberate;
    return d;
  }
  bool CollisionImminent(const Pose2d&, Twist2d, const LayeredCostmap&) const override {
    return false;
  }
};
class TypedController final : public LocalController {
 public:
  LocalControlResult result;
  bool CollisionImminent(const Pose2d&, Twist2d, const LayeredCostmap&) const override {
    return false;
  }

 protected:
  LocalControlResult ComputeRequest(const LocalControlRequest&) const override { return result; }
};
}  // namespace
int main() {
  using namespace navigation2d;
  test::AlgorithmContractFixture fixture;
  auto map = fixture.Map();
  const auto pose = MakePose2d(1., 1., 0.);
  const Path path{pose, MakePose2d(2., 1., 0.)};
  const std::vector<PredictedObstacle> predictions;
  LocalControlRequest request{path, pose, {}, map, predictions};
  LegacyController controller;
  controller.run = [] { return Twist2d{.2, .3}; };
  auto result = controller.ComputeWithContext(request);
  assert(result.has_command() && result.command.linear == .2 && result.command.angular == .3);
  controller.run = [] { return Twist2d{}; };
  result = controller.ComputeWithContext(request);
  assert(result.status == LocalControlStatus::kSuccess && result.has_command());
  controller.deliberate = true;
  assert(controller.ComputeWithContext(request).status == LocalControlStatus::kIntentionalStop);
  for (double value :
       {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    controller.run = [=] { return Twist2d{value, 0.}; };
    result = controller.ComputeWithContext(request);
    assert(result.status == LocalControlStatus::kInvalidOutput && result.command.linear == 0.);
  }
  controller.run = []() -> Twist2d { throw std::logic_error("solver failed"); };
  result = controller.ComputeWithContext(request);
  assert(result.status == LocalControlStatus::kBackendError && result.reason == "solver failed");
  controller.run = []() -> Twist2d { throw 17; };
  assert(controller.ComputeWithContext(request).status == LocalControlStatus::kBackendError);
  const int calls = controller.calls;
  std::stop_source cancelled;
  cancelled.request_stop();
  request.budget.cancellation = cancelled.get_token();
  assert(controller.ComputeWithContext(request).status == LocalControlStatus::kCancelled);
  request.budget = {};
  request.budget.deadline = std::chrono::steady_clock::now();
  assert(controller.ComputeWithContext(request).status == LocalControlStatus::kBudgetExceeded);
  assert(controller.calls == calls);
  request.budget = {};
  request.measured_velocity.linear = std::numeric_limits<double>::quiet_NaN();
  assert(controller.ComputeWithContext(request).status == LocalControlStatus::kInvalidInput);
  assert(controller.calls == calls);
  request.measured_velocity = {};
  std::stop_source during;
  request.budget.cancellation = during.get_token();
  controller.run = [&] {
    during.request_stop();
    return Twist2d{.3, .5};
  };
  result = controller.ComputeWithContext(request);
  assert(result.status == LocalControlStatus::kCancelled && result.command.linear == 0.);
  request.budget = {};
  TypedController typed;
  typed.result = {LocalControlStatus::kInfeasible, {.4, .7}, "no feasible command"};
  result = typed.ComputeWithContext(request);
  assert(!result.has_command() && result.command.linear == 0. && result.command.angular == 0.);
  typed.result = {LocalControlStatus::kIntentionalStop, {.4, .7}, "bad stop"};
  assert(typed.ComputeWithContext(request).status == LocalControlStatus::kInvalidOutput);
  typed.result = {LocalControlStatus::kIntentionalStop, {}, "holding"};
  assert(typed.ComputeWithContext(request).status == LocalControlStatus::kIntentionalStop);
  // A deliberately slow diagnostic read must not enter the backend-hook timer.
  // The inequality uses observed event ordering rather than a machine speed threshold.
  controller.deliberate = false;
  controller.run = [] { return Twist2d{.1, 0.}; };
  std::chrono::steady_clock::time_point diagnostic_started;
  controller.diagnostic_action = [&] {
    diagnostic_started = std::chrono::steady_clock::now();
    const auto until = diagnostic_started + std::chrono::microseconds(200);
    while (std::chrono::steady_clock::now() < until) {
    }
  };
  const auto before_request = std::chrono::steady_clock::now();
  result = controller.ComputeWithContext(request);
  const double before_diagnostics_us =
      std::chrono::duration<double, std::micro>(diagnostic_started - before_request).count();
  assert(result.has_command() && result.backend_duration_us >= 0.);
  assert(result.backend_duration_us <= before_diagnostics_us);
}
