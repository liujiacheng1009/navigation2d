#include "navigation2d/application/exploration/execution_command.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
Twist2d ExecutionCommand(Twist2d command, bool safety_held,
                         const std::optional<EntryApproach>& approach,
                         const ExecutionCommandEvidence& evidence) {
  if (!safety_held && approach) {
    const auto& [mouth, target, robot] = *approach;
    const double mouth_range = std::hypot(robot.x - mouth.frontier_x, robot.y - mouth.frontier_y);
    const double goal_dist = std::hypot(robot.x - target.x, robot.y - target.y);
    const double front = evidence.CommandFrontRange();
    if (mouth_range < 1.20 && goal_dist > kPlanningRadius() && std::abs(command.linear) < .12 &&
        (front <= 0. || front >= kPlanningRadius()) &&
        !evidence.CommandSegmentCutsWall(robot.x, robot.y, target)) {
      const double bearing = std::atan2(target.y - robot.y, target.x - robot.x);
      const double delta = bearing - robot.yaw;
      const double error = std::atan2(std::sin(delta), std::cos(delta));
      if (std::abs(error) < .90) {
        command.linear = .16;
        command.angular = std::clamp(1.6 * error, -.70, .70);
      } else {
        command.linear = 0.;
        command.angular = std::clamp(1.9 * error, -.90, .90);
      }
    }
  }
  if (command.linear > 0.) {
    const double front = evidence.CommandFrontRange();
    if (HitInsideBody(front)) {
      command.linear = 0.;
      if (std::abs(command.angular) < .15) {
        command.angular = 0.;
      }
    }
  }
  return command;
}
}  // namespace navigation2d::exploration
