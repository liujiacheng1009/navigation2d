#include "navigation2d/application/docking_execution.h"

#include <cmath>
#include <utility>
namespace navigation2d {
DockingCycle DockingExecution::Advance(ExecutionWatchdog& watchdog, CommandSafetyPipeline& safety,
                                       const DockingInput& input) {
  const auto& robot = input.robot;
  const auto servo =
      servo_.Advance(robot.pose, input.goal, input.goal_distance_m, input.previous_published);
  watchdog.NoteDocking(servo.distance_progress, servo.yaw_progress, robot.timestamp_s);
  if (servo.succeeded) {
    return {true, {}, NavigationFailure::kNone};
  }
  auto motion = EvaluateMotion(safety, robot.pose, servo.command, robot.timestamp_s);
  auto failure = watchdog.ObserveStall(
      {robot.pose, 0., robot.measured_velocity, servo.yaw_error_rad, robot.timestamp_s,
       motion.reached_base, std::abs(motion.monitored.command.linear) > 1e-3},
      StallMode::kDocking);
  if (failure == NavigationFailure::kNone) {
    failure = watchdog.DockingTimeout(robot.timestamp_s);
  }
  return {false, std::move(motion), failure};
}
}  // namespace navigation2d
