#include "navigation2d/application/alignment_execution.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "navigation2d/planning/path_geometry.h"
namespace navigation2d {
AlignmentCycle AlignmentExecution::Advance(ExecutionWatchdog& watchdog,
                                           CommandSafetyPipeline& safety,
                                           const AlignmentInput& input) const {
  const auto& robot = input.robot;
  const double error =
      planning_internal::InitialHeadingError(input.path, input.metadata.headings, robot.pose);
  const double score = std::abs(error) + planning_internal::DistanceToPath(input.path, robot.pose);
  if (watchdog.ObserveAlignment(score, error, input.path_progress_m, robot.timestamp_s)) {
    return {};
  }
  const double dv = settings_.linear_acceleration_mps2 * settings_.period_s;
  const double dw = settings_.angular_acceleration_rps2 * settings_.period_s;
  Twist2d command{
      0., std::clamp(2.2 * error, -settings_.maximum_angular_rps, settings_.maximum_angular_rps)};
  command.linear =
      std::clamp(0., input.previous_published.linear - dv, input.previous_published.linear + dv);
  command.angular = std::clamp(command.angular, input.previous_published.angular - dw,
                               input.previous_published.angular + dw);
  watchdog.ChargeRotation(command.angular);
  auto motion = EvaluateMotion(safety, robot.pose, command, robot.timestamp_s);
  auto failure = watchdog.ObserveStall(
      {robot.pose, input.path_progress_m, robot.measured_velocity, error, robot.timestamp_s,
       motion.reached_base, std::abs(motion.monitored.command.linear) > 1e-3},
      StallMode::kAlignment);
  if (failure == NavigationFailure::kNone) {
    failure = watchdog.CheckRotation(RotationStage::kAlignment);
  }
  return {std::move(motion), failure};
}
}  // namespace navigation2d
