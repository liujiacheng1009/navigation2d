#include <cmath>
#include <exception>
#include <stdexcept>

#include "navigation2d/control/local_controller.h"
namespace navigation2d {
Twist2d LocalController::Compute(const Path&, const Pose2d&, Twist2d, const LayeredCostmap&,
                                const std::vector<PredictedObstacle>&) const {
  throw std::logic_error("controller does not implement the legacy Compute API");
}
LocalControlResult LocalController::ComputeWithContext(const LocalControlRequest& request) const {
  const auto stopped = [](AlgorithmStop stop) {
    return LocalControlResult{
        stop == AlgorithmStop::kCancelled ? LocalControlStatus::kCancelled
                                          : LocalControlStatus::kBudgetExceeded,
        {},
        stop == AlgorithmStop::kCancelled ? "control cancelled" : "control budget exceeded",
        {}};
  };
  if (const auto stop = request.budget.Check(); stop != AlgorithmStop::kNone) {
    return stopped(stop);
  }
  if (!request.pose.matrix().allFinite() || !std::isfinite(request.measured_velocity.linear) ||
      !std::isfinite(request.measured_velocity.angular)) {
    return {LocalControlStatus::kInvalidInput, {}, "invalid control pose or measured velocity", {}};
  }
  try {
    auto result = ComputeRequest(request);
    const auto command = result.command;
    if (const auto stop = request.budget.Check(); stop != AlgorithmStop::kNone) {
      return stopped(stop);
    }
    if (!std::isfinite(command.linear) || !std::isfinite(command.angular)) {
      return {
          LocalControlStatus::kInvalidOutput, {}, "controller returned non-finite velocity", {}};
    }
    if (!result.has_command()) {
      result.command = {};
    }
    if (result.status == LocalControlStatus::kIntentionalStop &&
        (command.linear != 0. || command.angular != 0.)) {
      return {LocalControlStatus::kInvalidOutput, {}, "intentional stop returned motion", {}};
    }
    return result;
  } catch (const std::exception& error) {
    return {LocalControlStatus::kBackendError, {}, error.what(), {}};
  } catch (...) {
    return {LocalControlStatus::kBackendError, {}, "controller threw a non-standard exception", {}};
  }
}
LocalControlResult LocalController::ComputeRequest(const LocalControlRequest& request) const {
  LocalControlResult result;
  result.command = Compute(request.path, request.pose, request.measured_velocity, request.costmap,
                           request.dynamic_obstacles);
  result.diagnostics = Diagnostics();
  if (result.command.linear == 0. && result.command.angular == 0. &&
      result.diagnostics->intentional_stop) {
    result.status = LocalControlStatus::kIntentionalStop;
  }
  return result;
}
}  // namespace navigation2d
