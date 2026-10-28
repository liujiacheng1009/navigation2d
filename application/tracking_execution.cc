#include "navigation2d/application/tracking_execution.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/application/command_safety_pipeline.h"
#include "navigation2d/application/navigation_algorithms.h"
#include "navigation2d/planning/path_geometry.h"
namespace navigation2d {
TrackingCycle RunTrackingCycle(NavigationAlgorithms& algorithms, ExecutionWatchdog& watchdog,
                               CommandSafetyPipeline& safety, const TrackingInput& input) {
  const auto& robot = input.robot;
  const double error =
      planning_internal::InitialHeadingError(input.path, input.metadata.headings, robot.pose);
  TrackingCycle result{
      .control = algorithms.Compute({input.path, robot.pose, robot.measured_velocity, input.costmap,
                                     safety.predictions(), input.budget, &input.metadata})};
  if (!result.control.has_command()) {
    return result;
  }
  watchdog.ChargeController(result.control.command);
  result.motion = EvaluateMotion(safety, robot.pose, result.control.command, robot.timestamp_s);
  const auto& motion = *result.motion;
  const auto published = motion.monitored.command;
  const bool deliberate = result.control.diagnostics->intentional_stop;
  result.failure = watchdog.ObserveStall(
      {robot.pose, input.path_progress_m, robot.measured_velocity, error, robot.timestamp_s,
       motion.reached_base && !deliberate, std::abs(published.linear) > 1e-3},
      StallMode::kTracking);
  if (result.failure != NavigationFailure::kNone) {
    return result;
  }
  result.failure =
      watchdog.ObserveController(motion.requested_motion, motion.safety_stopped, deliberate);
  if (result.failure != NavigationFailure::kNone) {
    return result;
  }
  result.failure = watchdog.CheckRotation(RotationStage::kAfterController);
  if (result.failure != NavigationFailure::kNone) {
    return result;
  }
  const double raw = motion.monitored.time_to_collision_s;
  const double dynamic = safety.DynamicTtc(robot.pose, published);
  result.minimum_ttc_s = raw > 0. && dynamic > 0. ? std::min(raw, dynamic) : std::max(raw, dynamic);
  return result;
}
}  // namespace navigation2d
