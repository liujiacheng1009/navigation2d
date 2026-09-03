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
  std::vector<Eigen::Vector2d> blind_zone_points;
  points.reserve(scan.ranges.size());
  blind_zone_points.reserve(scan.ranges.size());
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const double range = scan.ranges[index];
    const double angle = scan.angle_min + index * scan.angle_increment;
    // Gazebo's GPU lidar (and several real drivers) encode a return closer
    // than range_min as -inf.  It is not a free/no-return beam: the obstacle
    // is inside the sensor's measurement envelope.  Preserve a conservative
    // point at range_min so the monitor can stop a command that closes on the
    // blind zone.  Finite sub-minimum values are handled the same way.
    const bool below_min = (std::isinf(range) && range < 0.) ||
        (std::isfinite(range) && range >= 0. && range < scan.range_min);
    if (below_min) {
      if (scan.range_min > 0.)
        blind_zone_points.emplace_back(scan.range_min * std::cos(angle),
                                       scan.range_min * std::sin(angle));
      continue;
    }
    if (!std::isfinite(range) || range > scan.range_max) continue;
    points.emplace_back(range * std::cos(angle), range * std::sin(angle));
  }
  UpdatePoints(sensor_pose, std::move(points));
  blind_zone_points_.clear();
  blind_zone_points_.reserve(blind_zone_points.size());
  const double cosine = std::cos(Yaw(sensor_pose)), sine = std::sin(Yaw(sensor_pose));
  for (const auto& point : blind_zone_points)
    blind_zone_points_.emplace_back(
        X(sensor_pose) + cosine * point.x() - sine * point.y(),
        Y(sensor_pose) + sine * point.x() + cosine * point.y());
}

void CollisionMonitor::UpdatePointCloud(const Pose2d& sensor_pose, const PointCloud2d& cloud) {
  blind_zone_points_.clear();
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
  // Laser points are continuous measurements, but the route/controller
  // contract is evaluated on an occupancy raster.  Match the planner's
  // one-cell half-diagonal guard so a command cannot enter the physical wall
  // during the small gap between the raster boundary and a real contact.
  const double collision_radius = config_.robot_radius +
      std::sqrt(.5) * config_.map_resolution;
  double ttc = std::numeric_limits<double>::infinity();
  bool blind_zone_closing = false;
  Pose2d projected = robot_pose;
  for (double time = config_.control_period;
       time <= config_.collision_monitor_approach_horizon + 1e-9;
       time += config_.control_period) {
    projected = Integrate(projected, command, config_.control_period);
    const bool collision = std::any_of(points_.begin(), points_.end(), [&](const auto& point) {
      const double initial_distance = (point - robot_pose.translation()).norm();
      const double projected_distance = (point - projected.translation()).norm();
      // Footprint clearing: a static obstacle cannot physically occupy the
      // robot's current solid body. Returns already inside the footprint are
      // self/noise/contact discretization and must not permanently latch the
      // base. The monitor guards newly swept space only.
      return initial_distance > collision_radius &&
             projected_distance <= collision_radius;
    });
    if (collision) { ttc = time; break; }
  }

  // A below-minimum return may already be close enough that the normal
  // projected-point test cannot see a new crossing (or may disappear from
  // the next scan).  Stop only when the commanded translation closes on the
  // return; in-place rotation and motion away from it remain executable.
  if (std::abs(command.linear) > 1e-9 && !blind_zone_points_.empty()) {
    const Pose2d one_step = Integrate(robot_pose, command, config_.control_period);
    blind_zone_closing = std::any_of(
        blind_zone_points_.begin(), blind_zone_points_.end(), [&](const auto& point) {
          const double initial_distance = (point - robot_pose.translation()).norm();
          const double projected_distance = (point - one_step.translation()).norm();
          return initial_distance > collision_radius + 1e-3 &&
                 projected_distance + 1e-5 < initial_distance;
        });
    if (blind_zone_closing) ttc = std::min(ttc, config_.control_period);
  }

  CollisionMonitorAction requested = CollisionMonitorAction::kNone;
  // A lidar return beside (or slightly behind) the footprint is not a
  // collision with the *commanded* motion.  Treating every such return as an
  // emergency stop permanently latches the robot at shelf ends and doorway
  // jambs: the controller is unable to turn or move away, so the recovery
  // tree times out despite a valid global path.  The static/local costmap
  // still rejects any swept-footprint collision; the live monitor's stop
  // authority is deliberately limited to a collision predicted along the
  // command trajectory on the next control step.
  if (blind_zone_closing)
    requested = CollisionMonitorAction::kBlindZoneStop;
  else if (ttc <= config_.control_period)
    requested = CollisionMonitorAction::kStop;
  // Side walls in a narrow but traversable doorway must not throttle the
  // robot indefinitely. Slow down only when the commanded swept footprint
  // actually approaches a collision; retain the all-direction emergency stop.
  else if (std::isfinite(ttc))
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
  if (latched_action_ == CollisionMonitorAction::kStop ||
      latched_action_ == CollisionMonitorAction::kBlindZoneStop) command = {};
  else if (latched_action_ == CollisionMonitorAction::kSlowdown) {
    command.linear *= config_.collision_monitor_slowdown_ratio;
    command.angular *= config_.collision_monitor_slowdown_ratio;
  }
  return {command, latched_action_, min_distance,
          std::isfinite(ttc) ? ttc : 0.};
}

}  // namespace navigation2d
