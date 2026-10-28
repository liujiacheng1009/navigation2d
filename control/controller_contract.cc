#include <chrono>
#include <cmath>
#include <exception>
#include <stdexcept>

#include "navigation2d/control/local_controller.h"
#include "navigation2d/costmap/coordinate_validation.h"
namespace navigation2d {
Twist2d LocalController::Compute(const Path&, const Pose2d&, Twist2d, const LayeredCostmap&,
                                 const std::vector<PredictedObstacle>&) const {
  throw std::logic_error("controller does not implement the legacy Compute API");
}
LocalControlResult LocalController::ComputeWithContext(const LocalControlRequest& request) const {
  const auto stopped = [](AlgorithmStop stop, double backend_us = 0.) {
    return LocalControlResult{
        stop == AlgorithmStop::kCancelled ? LocalControlStatus::kCancelled
                                          : LocalControlStatus::kBudgetExceeded,
        {},
        stop == AlgorithmStop::kCancelled ? "control cancelled" : "control budget exceeded",
        {},
        backend_us};
  };
  if (const auto stop = request.budget.Check(); stop != AlgorithmStop::kNone) {
    return stopped(stop);
  }
  if (!costmap_internal::IndexablePose(request.costmap.grid(), request.pose) ||
      !std::isfinite(request.measured_velocity.linear) ||
      !std::isfinite(request.measured_velocity.angular)) {
    return {LocalControlStatus::kInvalidInput, {}, "invalid control pose or measured velocity", {}};
  }
  const auto started = std::chrono::steady_clock::now();
  double backend_us = 0.;
  bool hook_returned = false;
  const auto failed_duration = [&] {
    return hook_returned ? backend_us
                         : std::chrono::duration<double, std::micro>(
                               std::chrono::steady_clock::now() - started)
                               .count();
  };
  try {
    auto result = ComputeRequest(request);
    backend_us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - started)
            .count();
    hook_returned = true;
    result.backend_duration_us = backend_us;
    const auto command = result.command;
    if (const auto stop = request.budget.Check(); stop != AlgorithmStop::kNone) {
      return stopped(stop, backend_us);
    }
    if (!std::isfinite(command.linear) || !std::isfinite(command.angular)) {
      return {LocalControlStatus::kInvalidOutput,
              {},
              "controller returned non-finite velocity",
              {},
              backend_us};
    }
    if (result.status == LocalControlStatus::kNotRun) {
      result.status = LocalControlStatus::kInvalidOutput;
      result.reason = "controller returned no outcome";
    }
    if (!result.has_command()) {
      result.command = {};
    }
    if (result.status == LocalControlStatus::kIntentionalStop &&
        (command.linear != 0. || command.angular != 0.)) {
      return {LocalControlStatus::kInvalidOutput,
              {},
              "intentional stop returned motion",
              {},
              backend_us};
    }
    if (result.has_command() && !result.diagnostics) {
      result.diagnostics = Diagnostics();
    }
    if (result.status == LocalControlStatus::kSuccess && command.linear == 0. &&
        command.angular == 0. && result.diagnostics && result.diagnostics->intentional_stop) {
      result.status = LocalControlStatus::kIntentionalStop;
    }
    if (result.status == LocalControlStatus::kIntentionalStop) {
      result.diagnostics->intentional_stop = true;
    }
    if (const auto stop = request.budget.Check(); stop != AlgorithmStop::kNone) {
      return stopped(stop, backend_us);
    }
    return result;
  } catch (const std::exception& error) {
    return {LocalControlStatus::kBackendError, {}, error.what(), {}, failed_duration()};
  } catch (...) {
    return {LocalControlStatus::kBackendError,
            {},
            "controller threw a non-standard exception",
            {},
            failed_duration()};
  }
}
LocalControlResult LocalController::ComputeRequest(const LocalControlRequest& request) const {
  LocalControlResult result;
  result.command = Compute(request.path, request.pose, request.measured_velocity, request.costmap,
                           request.dynamic_obstacles);
  return result;
}
}  // namespace navigation2d
