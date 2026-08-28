// Architecture adapted from Nav2 Collision Monitor, Apache-2.0.
#include "navigation2d/control/collision_monitor.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation2d {
namespace {

Pose2d Integrate(const Pose2d& pose, const Twist2d& command, double dt) {
  const double yaw = Yaw(pose);
  const double next_yaw = yaw + command.angular * dt;
  if (std::abs(command.angular) < 1e-9)
    return MakePose2d(X(pose) + command.linear * std::cos(yaw) * dt,
                      Y(pose) + command.linear * std::sin(yaw) * dt, next_yaw);
  const double radius = command.linear / command.angular;
  return MakePose2d(X(pose) + radius * (std::sin(next_yaw) - std::sin(yaw)),
                    Y(pose) - radius * (std::cos(next_yaw) - std::cos(yaw)), next_yaw);
}

}  // namespace

void CollisionMonitor::UpdatePoints(const Pose2d& sensor_pose,
                                    std::vector<Eigen::Vector2d> points) {
  points_.clear();
  points_.reserve(points.size());
  const double cosine = std::cos(Yaw(sensor_pose)), sine = std::sin(Yaw(sensor_pose));
  for (const auto& point : points)
    points_.emplace_back(X(sensor_pose) + cosine * point.x() - sine * point.y(),
                         Y(sensor_pose) + sine * point.x() + cosine * point.y());
  observation_pending_ = true;
}

void CollisionMonitor::UpdateLaserScan(const Pose2d& sensor_pose, const LaserScan& scan) {
  std::vector<Eigen::Vector2d> points;
  points.reserve(scan.ranges.size());
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const double range = scan.ranges[index];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) continue;
    const double angle = scan.angle_min + index * scan.angle_increment;
    points.emplace_back(range * std::cos(angle), range * std::sin(angle));
  }
  UpdatePoints(sensor_pose, std::move(points));
}

void CollisionMonitor::UpdatePointCloud(const Pose2d& sensor_pose, const PointCloud2d& cloud) {
  std::vector<Eigen::Vector2d> points;
  points.reserve(cloud.points.size());
  for (const auto& point : cloud.points)
    if (point.allFinite() && point.norm() <= cloud.range_max) points.push_back(point);
  UpdatePoints(sensor_pose, std::move(points));
}

CollisionMonitorResult CollisionMonitor::Filter(const Pose2d& robot_pose, Twist2d command,
                                                 double timestamp) {
  if (!config_.collision_monitor_enabled) return {command};
  if (observation_pending_) {
    last_observation_time_ = timestamp;
    observation_pending_ = false;
    has_observation_ = true;
  }
  if (!has_observation_ ||
      timestamp - last_observation_time_ > config_.collision_monitor_source_timeout)
    return {{}, CollisionMonitorAction::kSourceTimeout, 0., 0.};

  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto& point : points_)
    min_distance = std::min(min_distance,
        std::max(0., (point - robot_pose.translation()).norm() - config_.robot_radius));
  double ttc = std::numeric_limits<double>::infinity();
  Pose2d projected = robot_pose;
  for (double time = config_.control_period;
       time <= config_.collision_monitor_approach_horizon + 1e-9;
       time += config_.control_period) {
    projected = Integrate(projected, command, config_.control_period);
    const bool collision = std::any_of(points_.begin(), points_.end(), [&](const auto& point) {
      return (point - projected.translation()).norm() <= config_.robot_radius;
    });
    if (collision) { ttc = time; break; }
  }

  CollisionMonitorAction requested = CollisionMonitorAction::kNone;
  if (min_distance <= config_.collision_monitor_stop_distance ||
      ttc <= config_.control_period)
    requested = CollisionMonitorAction::kStop;
  else if (min_distance <= config_.collision_monitor_slowdown_distance ||
           std::isfinite(ttc))
    requested = CollisionMonitorAction::kSlowdown;

  if (requested != CollisionMonitorAction::kNone) {
    ++trigger_count_; release_count_ = 0;
    if (trigger_count_ >= config_.collision_monitor_trigger_cycles)
      latched_action_ = requested;
  } else {
    trigger_count_ = 0; ++release_count_;
    if (release_count_ >= config_.collision_monitor_release_cycles)
      latched_action_ = CollisionMonitorAction::kNone;
  }
  if (latched_action_ == CollisionMonitorAction::kStop) command = {};
  else if (latched_action_ == CollisionMonitorAction::kSlowdown) {
    command.linear *= config_.collision_monitor_slowdown_ratio;
    command.angular *= config_.collision_monitor_slowdown_ratio;
  }
  return {command, latched_action_, min_distance,
          std::isfinite(ttc) ? ttc : 0.};
}

}  // namespace navigation2d
