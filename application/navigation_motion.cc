#include "navigation2d/application/navigation_motion.h"

#include <cmath>

#include "navigation2d/application/command_safety_pipeline.h"
#include "navigation2d/application/navigation_state.h"
namespace navigation2d {
EvaluatedMotion EvaluateMotion(CommandSafetyPipeline& safety, const Pose2d& pose, Twist2d requested,
                               double timestamp_s) {
  EvaluatedMotion result;
  result.requested = requested;
  result.requested_motion = std::abs(requested.linear) > 1e-4 || std::abs(requested.angular) > 1e-4;
  result.monitored = safety.Filter(pose, requested, timestamp_s);
  const auto published = result.monitored.command;
  result.safety_stopped = result.requested_motion && std::abs(published.linear) <= 1e-4 &&
                          std::abs(published.angular) <= 1e-4;
  result.reached_base = result.requested_motion &&
                        (std::abs(published.linear) > 1e-4 || std::abs(published.angular) > 1e-4) &&
                        !result.safety_stopped;
  return result;
}
void CaptureMotion(NavigationState& snapshot, const EvaluatedMotion& motion) {
  snapshot.requested_command = motion.requested;
  snapshot.controller_commanded_motion = motion.requested_motion;
  snapshot.published_command = motion.monitored.command;
  snapshot.collision_monitor_action = motion.monitored.action;
  snapshot.safety_stopped_motion = motion.safety_stopped;
}
}  // namespace navigation2d
