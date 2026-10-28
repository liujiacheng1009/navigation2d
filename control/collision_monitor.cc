// Architecture adapted from Nav2 Collision Monitor, Apache-2.0.
#include "navigation2d/control/collision_monitor.h"
#include "navigation2d/control/collision_monitor_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation2d {
using collision_internal::CommandedExtent;
using collision_internal::Integrate;
using collision_internal::PointInsideOrNearFootprint;
using collision_internal::ShellReturn;
using collision_internal::SlowdownScale;
using collision_internal::StopCorridorHalfWidth;
using collision_internal::TravelCorridorHalfWidth;


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

void CollisionMonitor::UpdateDepthPoints(const Pose2d& pose,
    const std::vector<Eigen::Vector2d>& points, double timestamp) {
  depth_.Update(pose, points, timestamp, config_);
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

  depth_.Refresh(robot_pose, timestamp, config_);
  struct SafetyReturn {
    Eigen::Vector2d point;
    double initial_distance = 0.;
  };
  std::vector<SafetyReturn> returns;
  returns.reserve(points_.size() + depth_.points().size() + depth_.held().size());
  for (const auto& point : points_) returns.push_back({point, 0.});
  for (const auto& point : depth_.points()) returns.push_back({point, 0.});
  for (const auto& point : depth_.held()) returns.push_back({point, 0.});

  double min_distance = std::numeric_limits<double>::infinity();
  // One immutable robot pose per filter frame. Reuse each measured distance
  // across swept predictions; projected distances still change at every step.
  for (auto& sample : returns) {
    sample.initial_distance = (sample.point - robot_pose.translation()).norm();
    min_distance = std::min(min_distance,
        std::max(0., sample.initial_distance - config_.robot_radius));
  }
  // Returns are appended as lidar, live depth, then held depth. Deriving the
  // source from this boundary leaves room for the cached double without an
  // additional per-return flag or a second allocation.
  const auto lidar_returns = points_.size();
  // Laser points are continuous measurements, but the route/controller
  // contract is evaluated on an occupancy raster.  Match the planner's
  // one-cell half-diagonal guard so a command cannot enter the physical wall
  // during the small gap between the raster boundary and a real contact.
  // Keep the live swept-footprint check slightly farther from sparse thin
  // returns such as table legs than the raster footprint alone. This margin
  // is safety clearance, not an inflated robot geometry.
  const double collision_radius = config_.robot_radius +
      std::sqrt(.5) * config_.map_resolution + .10;
  const double footprint_margin = std::sqrt(.5) * config_.map_resolution + .05;
  const double travel_half_width = TravelCorridorHalfWidth(config_);
  const double stop_half_width = StopCorridorHalfWidth(config_);
  const double commanded_extent = CommandedExtent(config_, command.linear);
  bool emergency_stop = false;
  double ttc = std::numeric_limits<double>::infinity();
  double slowdown_clearance = std::numeric_limits<double>::infinity();
  bool blind_zone_closing = false;
  bool slowdown_closing = false;
  // Doorway alignment often contains a tiny residual linear command while
  // the base is primarily rotating. Do not classify side-jamb returns as a
  // translational collision during that alignment phase.
  const bool translating = std::abs(command.linear) > .03;
  Pose2d projected = robot_pose;
  for (double time = config_.control_period;
       translating &&
       time <= config_.collision_monitor_approach_horizon + 1e-9;
       time += config_.control_period) {
    projected = Integrate(projected, command, config_.control_period);
    const double command_heading = Yaw(robot_pose);
    const Eigen::Vector2d heading(std::cos(command_heading), std::sin(command_heading));
    bool predicted_contact = false;
    for (std::size_t sample_index = 0; sample_index < returns.size(); ++sample_index) {
      const auto& sample = returns[sample_index];
      const bool depth = sample_index >= lidar_returns;
      const Eigen::Vector2d& point = sample.point;
      if (ShellReturn(point, robot_pose, config_, sample.initial_distance)) continue;
      const double initial_distance = sample.initial_distance;
      const double projected_distance = (point - projected.translation()).norm();
      const Eigen::Vector2d relative = point - robot_pose.translation();
      const double longitudinal = relative.dot(heading);
      const double lateral = std::abs(relative.x() * heading.y() -
                                      relative.y() * heading.x());
      // Only returns in the commanded travel corridor participate in the
      // swept translation test.  Door jambs beside the chassis must not be
      // treated as a collision merely because the command also rotates.
      // travel_half_width is the body plus the footprint edge margin.
      // collision_radius is wider by 0.10 m and is only an along-track stop
      // distance; using it here latches slowdown on every parallel wall.
      const bool in_travel_corridor =
          ((command.linear > 0. && longitudinal > 0.) ||
           (command.linear < 0. && longitudinal < 0.)) &&
          lateral <= travel_half_width;
      const bool closing = projected_distance + 1e-5 < initial_distance;
      const double clearance = initial_distance - config_.robot_radius;
      // Depth is a noisy near field. It stops inside the bumper gap and does
      // not scale speed the way a lidar return in the 0.45 m zone does.
      if (!depth && in_travel_corridor && closing && clearance > 0. &&
          clearance <= config_.collision_monitor_slowdown_distance) {
        slowdown_clearance = std::min(slowdown_clearance, clearance);
        slowdown_closing = true;
      }
      const bool projected_footprint_collision =
          config_.footprint.size() >= 3
              ? PointInsideOrNearFootprint(point, projected, config_, footprint_margin)
              : projected_distance <= collision_radius;
      // Stop on the gap in front of the bumper, not on radial distance from
      // the robot centre. A return beside the front corner is only ~0.08 m
      // from the 0.22 m planning circle, which is inside stop_distance, while
      // the bumper itself still has a clear lane. Radial clearance stopped
      // that lane and the zero command then held for the rest of the goal.
      // The stop corridor is the body width; the wider travel corridor only
      // slows down. Scan every return before leaving this step so a side
      // graze cannot hide a bumper obstacle.
      const bool in_stop_corridor =
          ((command.linear > 0. && longitudinal > 0.) ||
           (command.linear < 0. && longitudinal < 0.)) &&
          lateral <= stop_half_width;
      const double along_track_gap = std::abs(longitudinal) - commanded_extent;
      if (in_stop_corridor && closing && along_track_gap > -0.02 &&
          along_track_gap <= config_.collision_monitor_stop_distance)
        emergency_stop = true;
      // Footprint clearing must use the STOP corridor, not the wider travel
      // corridor. Door jambs sit in the travel band beside a 0.55–0.70 m mouth;
      // treating them as predicted_contact latched STOP (ttc≈one cycle) and
      // left the robot parked on the threshold despite a clear centerline.
      if (!depth && in_stop_corridor && initial_distance > collision_radius &&
          projected_footprint_collision)
        predicted_contact = true;
    }
    if (emergency_stop || predicted_contact) { ttc = time; break; }
  }

  // A below-minimum return may already be close enough that the normal
  // projected-point test cannot see a new crossing (or may disappear from
  // the next scan).  Stop only when the commanded translation closes on the
  // return; in-place rotation and motion away from it remain executable.
  if (std::abs(command.linear) > 1e-9 && !blind_zone_points_.empty()) {
    const Pose2d one_step = Integrate(robot_pose, command, config_.control_period);
    blind_zone_closing = std::any_of(
        blind_zone_points_.begin(), blind_zone_points_.end(), [&](const auto& point) {
          if (ShellReturn(point, robot_pose, config_)) return false;
          const double initial_distance = (point - robot_pose.translation()).norm();
          const double projected_distance = (point - one_step.translation()).norm();
          const Eigen::Vector2d relative = point - robot_pose.translation();
          const double heading_yaw = Yaw(robot_pose);
          const Eigen::Vector2d heading(std::cos(heading_yaw), std::sin(heading_yaw));
          const double longitudinal = relative.dot(heading);
          const double lateral = std::abs(relative.x() * heading.y() -
                                          relative.y() * heading.x());
          const bool in_travel_corridor =
              ((command.linear > 0. && longitudinal > 0.) ||
               (command.linear < 0. && longitudinal < 0.)) &&
              lateral <= collision_radius;
          return in_travel_corridor && initial_distance > collision_radius + 1e-3 &&
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
  else if (emergency_stop)
    requested = CollisionMonitorAction::kStop;
  // Side walls in a narrow but traversable doorway must not throttle the
  // robot indefinitely. Slow down only when the commanded swept footprint
  // actually approaches a collision; retain the all-direction emergency stop.
  else if (std::isfinite(ttc) || slowdown_closing)
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
      latched_action_ == CollisionMonitorAction::kBlindZoneStop) {
    // A safety filter must not turn an arc into an unvalidated pivot.
    // Replanning/recovery belongs to the controller, not this filter.
    command = {};
  }
  else if (latched_action_ == CollisionMonitorAction::kSlowdown) {
    const double requested_linear = command.linear;
    const double scale = SlowdownScale(slowdown_clearance, ttc, config_);
    command.linear *= scale;
    command.angular *= scale;
    // A flat 0.35 scale of an already regulated command fell through to a
    // 0.035 m/s floor. Recorded exploration sat on that floor and then
    // stopped commanding for the rest of the traverse. Do not publish
    // slower than the controller's regulated minimum when the unscaled
    // command was a real drive. The stop zone still zeroes the command.
    const double executable = std::max(.05, config_.regulated_min_speed);
    if (std::abs(requested_linear) >= executable &&
        std::abs(command.linear) < executable)
      command.linear = std::copysign(executable, requested_linear);
  }
  return {command, latched_action_, min_distance,
          std::isfinite(ttc) ? ttc : 0.};
}

}  // namespace navigation2d
