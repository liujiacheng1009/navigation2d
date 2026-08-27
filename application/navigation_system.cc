#include "navigation2/application/navigation_system.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "navigation2/control/regulated_pure_pursuit.h"
#include "navigation2/costmap/grid_2d.h"
#include "navigation2/planning/navfn_planner.h"

namespace navigation2d {
namespace { double Angle(double a) { return std::atan2(std::sin(a), std::cos(a)); } }

RunResult NavigationSystem::Run(const std::string& world_path, Pose2d pose, Pose2d goal,
                                const std::vector<ObstacleEvent>& obstacles) const {
  LayeredCostmap costmap(Grid2d::Load(world_path), config_);
  if (std::abs(costmap.grid().resolution() - config_.map_resolution) > 1e-9)
    throw std::runtime_error("map resolution does not match navigation configuration");
  // The master grid already contains the inscribed footprint and inflation
  // costs, so the planner must not apply the robot radius a second time.
  NavFnPlanner planner(0.);
  Path path = planner.Plan(costmap, pose, goal);
  RegulatedPurePursuit controller(config_);
  RunResult result; result.trajectory.push_back(pose);
  Velocity velocity;
  std::vector<bool> active(obstacles.size(), false);
  double last_progress_time = 0.;
  Pose2d last_progress_pose = pose;
  int recovery_steps_remaining = 0;
  const int max_steps = static_cast<int>(std::ceil(config_.max_navigation_duration /
                                                   config_.control_period));
  for (int step = 0; step < max_steps; ++step) {
    const double now = step * config_.control_period;
    bool map_changed = false;
    for (std::size_t i = 0; i < obstacles.size(); ++i) {
      const bool should_be_active = now >= obstacles[i].appear_s &&
          (obstacles[i].disappear_s < 0. || now < obstacles[i].disappear_s);
      if (should_be_active != active[i]) {
        active[i] = should_be_active; map_changed = true;
      }
    }
    LaserScan scan{-M_PI, 2. * M_PI / 360., .05, config_.raytrace_max_range,
                   std::vector<double>(360, config_.raytrace_max_range)};
    for (std::size_t ray = 0; ray < scan.ranges.size(); ++ray) {
      const double angle = pose.yaw + scan.angle_min + ray * scan.angle_increment;
      const double ux = std::cos(angle), uy = std::sin(angle);
      for (std::size_t i = 0; i < obstacles.size(); ++i) if (active[i]) {
        const double dx = obstacles[i].x - pose.x, dy = obstacles[i].y - pose.y;
        const double projection = dx * ux + dy * uy;
        const double perpendicular2 = dx * dx + dy * dy - projection * projection;
        const double kObstacleRadius = config_.dynamic_obstacle_radius;
        if (projection > 0. && perpendicular2 <= kObstacleRadius * kObstacleRadius) {
          const double range = projection - std::sqrt(kObstacleRadius * kObstacleRadius - perpendicular2);
          if (range >= scan.range_min) scan.ranges[ray] = std::min(scan.ranges[ray], range);
        }
      }
    }
    if (step % 2 == 0) {
      const auto before_scan = costmap.digest();
      costmap.UpdateObstacleLayer(pose, scan);
      map_changed = map_changed || costmap.digest() != before_scan;
    }
    if (map_changed || (now > 0. && std::fmod(now, config_.global_replan_period) < config_.control_period)) {
      try {
        path = planner.Plan(costmap, pose, goal); ++result.replans;
      } catch (const std::runtime_error&) {
        velocity = {}; ++result.emergency_stops;
        result.trajectory.push_back(pose); result.steps = step + 1; continue;
      }
    }
    const double error = std::hypot(goal.x - pose.x, goal.y - pose.y);
    if (error <= config_.goal_xy_tolerance) { result.status = "SUCCEEDED"; break; }
    Velocity command;
    if (recovery_steps_remaining > 0) {
      command = {std::max(-config_.max_reverse_velocity, config_.recovery_linear_velocity), 0.};
      if (controller.CollisionImminent(pose, command, costmap))
        command = {0., config_.recovery_angular_velocity};
      --recovery_steps_remaining;
    } else {
      command = controller.Compute(path, pose, velocity, costmap);
    }
    if (command.linear == 0. && command.angular == 0. &&
        (velocity.linear != 0. || velocity.angular != 0.)) ++result.emergency_stops;
    const double next_yaw = pose.yaw + command.angular * config_.control_period;
    Pose2d candidate;
    if (std::abs(command.angular) < 1e-9) {
      candidate = {pose.x + command.linear * std::cos(pose.yaw) * config_.control_period,
                   pose.y + command.linear * std::sin(pose.yaw) * config_.control_period, Angle(next_yaw)};
    } else {
      const double radius = command.linear / command.angular;
      candidate = {pose.x + radius * (std::sin(next_yaw) - std::sin(pose.yaw)),
                   pose.y - radius * (std::cos(next_yaw) - std::cos(pose.yaw)), Angle(next_yaw)};
    }
    bool physical_collision = costmap.grid().collides(candidate.x, candidate.y, config_.robot_radius);
    for (std::size_t i = 0; i < obstacles.size(); ++i) if (active[i])
      physical_collision = physical_collision ||
          std::hypot(candidate.x - obstacles[i].x, candidate.y - obstacles[i].y) <=
              config_.robot_radius + config_.dynamic_obstacle_radius;
    if (physical_collision) {
      ++result.collisions; velocity = {};
    } else { pose = candidate; velocity = command; }
    if (std::hypot(pose.x - last_progress_pose.x, pose.y - last_progress_pose.y) >= config_.progress_radius) {
      last_progress_pose = pose; last_progress_time = now;
    } else if (now - last_progress_time > config_.progress_timeout) {
      ++result.recoveries; last_progress_time = now;
      recovery_steps_remaining = static_cast<int>(
          std::ceil(config_.recovery_duration / config_.control_period));
    }
    result.trajectory.push_back(pose); result.steps = step + 1;
  }
  result.duration_s = result.steps * config_.control_period;
  result.goal_error_m = std::hypot(goal.x - pose.x, goal.y - pose.y);
  result.costmap_digest = costmap.digest();
  return result;
}

}  // namespace navigation2d
