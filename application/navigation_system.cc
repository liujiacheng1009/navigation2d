#include "navigation2/application/navigation_system.h"

#include <algorithm>
#include <cmath>
#include "navigation2/control/regulated_pure_pursuit.h"
#include "navigation2/mapping/grid_2d.h"
#include "navigation2/planning/navfn_planner.h"

namespace navigation2d {
namespace { double Angle(double a) { return std::atan2(std::sin(a), std::cos(a)); } }

RunResult NavigationSystem::Run(const std::string& world_path, Pose2d pose, Pose2d goal) const {
  constexpr double kDt = .06, kRobotRadius = .18, kGoalTolerance = .16;
  const Grid2d grid = Grid2d::Load(world_path);
  // One-cell tracking margin prevents the continuous controller from clipping
  // corners of a path that is collision-free only at grid cell centres.
  const Path path = NavFnPlanner(kRobotRadius + grid.resolution()).Plan(grid, pose, goal);
  RegulatedPurePursuit controller;
  RunResult result; result.trajectory.push_back(pose);
  double speed = 0.;
  for (int step = 0; step < 2000; ++step) {
    const double error = std::hypot(goal.x - pose.x, goal.y - pose.y);
    if (error <= kGoalTolerance) { result.status = "SUCCEEDED"; break; }
    const Velocity command = controller.Compute(path, pose, speed);
    const double next_yaw = pose.yaw + command.angular * kDt;
    Pose2d candidate;
    if (std::abs(command.angular) < 1e-9) {
      candidate = {pose.x + command.linear * std::cos(pose.yaw) * kDt,
                   pose.y + command.linear * std::sin(pose.yaw) * kDt, Angle(next_yaw)};
    } else {
      const double radius = command.linear / command.angular;
      candidate = {pose.x + radius * (std::sin(next_yaw) - std::sin(pose.yaw)),
                   pose.y - radius * (std::cos(next_yaw) - std::cos(pose.yaw)), Angle(next_yaw)};
    }
    if (grid.collides(candidate.x, candidate.y, kRobotRadius)) {
      ++result.collisions; speed = 0.;
    } else { pose = candidate; speed = command.linear; }
    result.trajectory.push_back(pose); result.steps = step + 1;
  }
  result.duration_s = result.steps * kDt;
  result.goal_error_m = std::hypot(goal.x - pose.x, goal.y - pose.y);
  return result;
}

}  // namespace navigation2d
