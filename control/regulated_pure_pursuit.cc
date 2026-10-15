#include "navigation2d/control/regulated_pure_pursuit.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace navigation2d {
namespace {
double Angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

double Smoothstep01(double t) {
  t = std::clamp(t, 0., 1.);
  return t * t * (3. - 2. * t);
}

// Local lethal clearance — full-map nearestLethalDistance is O(WH) per tick.
double LocalLethalClearance(const LayeredCostmap& costmap, double x, double y,
                            double search_radius) {
  const auto& grid = costmap.grid();
  const double resolution = grid.resolution();
  if (resolution <= 1e-9) return 0.;
  const double half = .5 * resolution;
  const int span = std::max(1, static_cast<int>(std::ceil(search_radius / resolution)));
  const auto [cx, cy] = grid.ToCell(x, y);
  double best = search_radius;
  bool found = false;
  for (int dy = -span; dy <= span; ++dy) {
    for (int dx = -span; dx <= span; ++dx) {
      const int ix = cx + dx, iy = cy + dy;
      if (ix < 0 || iy < 0 || ix >= grid.width() || iy >= grid.height()) continue;
      if (costmap.cost(ix, iy) != kLethal) continue;
      const auto [wx, wy] = grid.CellCenter(ix, iy);
      const double nearest_x = std::max(std::abs(x - wx) - half, 0.);
      const double nearest_y = std::max(std::abs(y - wy) - half, 0.);
      best = std::min(best, std::hypot(nearest_x, nearest_y));
      found = true;
    }
  }
  return found ? best : search_radius;
}

bool ForwardCorridorTrusted(const LayeredCostmap& costmap, const Pose2d& pose,
                            double robot_radius, double length, double half_width) {
  const double yaw = Yaw(pose);
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double lateral_c = -s, lateral_s = c;
  constexpr int kAlong = 4;
  for (int i = 1; i <= kAlong; ++i) {
    const double d = length * (static_cast<double>(i) / kAlong);
    for (const double lat : {0., half_width, -half_width}) {
      const double x = X(pose) + d * c + lat * lateral_c;
      const double y = Y(pose) + d * s + lat * lateral_s;
      if (costmap.lethal(x, y, robot_radius)) return false;
    }
  }
  return true;
}

double ClearanceSpeedScale(double margin, bool forward_trusted,
                           const NavigationConfig& config) {
  const double band = forward_trusted ? config.regulated_soft_clearance_trusted
                                      : config.regulated_soft_clearance;
  const double floor_scale = forward_trusted ? config.regulated_clearance_min_scale_trusted
                                             : config.regulated_clearance_min_scale;
  if (band <= 1e-9 || margin >= band) return 1.;
  if (margin <= 0.) return floor_scale;
  const double s = Smoothstep01(margin / band);
  return floor_scale + (1. - floor_scale) * s;
}
}  // namespace

Twist2d RegulatedPurePursuit::Compute(const Path& path, const Pose2d& pose, Twist2d current,
                                       const LayeredCostmap& costmap,
                                       const std::vector<PredictedObstacle>& dynamic_obstacles) const {
  (void)dynamic_obstacles;
  fallback_level_ = 0;
  maneuver_ = ControllerManeuver::kTracking;
  intentional_stop_ = false;
  if (path.empty()) return {};
  const bool path_replaced = path.size() != path_size_ || path.empty() ||
      std::hypot(X(path.front()) - path_start_x_, Y(path.front()) - path_start_y_) > .05 ||
      std::hypot(X(path.back()) - path_goal_x_, Y(path.back()) - path_goal_y_) > .05;
  if (path_replaced) {
    progress_index_ = 0;
    mode_ = PursuitMode::kTracking;
    recovery_heading_ = 0.;
    stopped_cycles_ = 0;
    aligned_cycles_ = 0;
    path_size_ = path.size();
    if (!path.empty()) {
      path_start_x_ = X(path.front()); path_start_y_ = Y(path.front());
      path_goal_x_ = X(path.back()); path_goal_y_ = Y(path.back());
    }
  }
  if (path.size() == 1) return {};
  // Project onto forward *segments*, rather than choosing a waypoint. Theta*
  // intentionally makes sparse paths; using its next vertex as the carrot
  // turns a 0.5 m lookahead into a multi-metre shortcut at every corner.
  // That shortcut is then rejected by collision checking, which used to look
  // like a mysterious RPP stall. Segment projection preserves the polyline's
  // arc-length topology and gives pure pursuit a genuinely local carrot.
  size_t nearest = std::min(progress_index_, path.size() - 2);
  double nearest_t = 0.;
  double nearest_distance = 1e30;
  double searched_arc = 0.;
  constexpr double kProjectionWindow = 1.25;
  for (size_t i = nearest; i + 1 < path.size(); ++i) {
    const auto start = path[i].translation();
    const auto segment = path[i + 1].translation() - start;
    const double squared_length = segment.squaredNorm();
    if (squared_length <= 1e-12) continue;
    if (searched_arc > kProjectionWindow) break;
    const double t = std::clamp((pose.translation() - start).dot(segment) /
                                    squared_length, 0., 1.);
    const double distance = (pose.translation() - (start + t * segment)).norm();
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = i;
      nearest_t = t;
    }
    searched_arc += std::sqrt(squared_length);
  }
  progress_index_ = std::max(progress_index_, nearest);
  const double nominal_lookahead = std::clamp(std::abs(current.linear) * config_.lookahead_time,
                                              config_.min_lookahead_distance,
                                              config_.max_lookahead_distance);
  const auto carrot_at = [&](double lookahead) {
    Eigen::Vector2d carrot = path[nearest].translation() +
        nearest_t * (path[nearest + 1].translation() - path[nearest].translation());
    double remaining = lookahead;
    size_t segment_index = nearest;
    double segment_t = nearest_t;
    while (segment_index + 1 < path.size()) {
      const auto start = path[segment_index].translation();
      const auto end = path[segment_index + 1].translation();
      const auto segment = end - start;
      const double length = segment.norm();
      if (length <= 1e-12) { ++segment_index; segment_t = 0.; continue; }
      const double available = (1. - segment_t) * length;
      if (remaining <= available) { carrot += (remaining / length) * segment; break; }
      carrot = end;
      remaining -= available;
      ++segment_index;
      segment_t = 0.;
    }
    return carrot;
  };
  const auto target_for = [&](const Eigen::Vector2d& carrot) {
    const double dx = carrot.x() - X(pose), dy = carrot.y() - Y(pose);
    const double heading_error = Angle(std::atan2(dy, dx) - Yaw(pose));
    if (std::abs(heading_error) > config_.rotate_to_heading_min_angle)
      return Twist2d{0., std::clamp(2.2 * heading_error, -config_.max_angular_velocity,
                                    config_.max_angular_velocity)};
    const double distance_to_goal = (path.back().translation() - pose.translation()).norm();
    double linear = std::min(config_.desired_linear_velocity,
                             std::max(config_.min_approach_velocity, distance_to_goal));
    linear *= std::max(.35, 1. - std::abs(heading_error));
    const double lateral = -std::sin(Yaw(pose)) * dx + std::cos(Yaw(pose)) * dy;
    const double curvature = 2. * lateral / std::max(.04, dx * dx + dy * dy);
    const double radius = std::abs(curvature) < 1e-6 ? 1e9 : std::abs(1. / curvature);
    if (radius < config_.regulated_min_radius)
      linear = std::max(config_.regulated_min_speed,
                        linear * radius / config_.regulated_min_radius);
    // Soft near-field regulation uses lethal clearance, not inflated cost.
    // Inflation remains for global planning preference; hard contact is still
    // CollisionImminent. When the short forward corridor is free, side walls
    // (door frames) keep a higher speed floor instead of crawling to 0.25×.
    const double search = config_.robot_radius +
        std::max(config_.regulated_soft_clearance, config_.regulated_soft_clearance_trusted) +
        .25;
    const double clearance = LocalLethalClearance(costmap, X(pose), Y(pose), search);
    const double margin = clearance - config_.robot_radius;
    const bool forward_trusted = ForwardCorridorTrusted(
        costmap, pose, config_.robot_radius, config_.regulated_forward_trust_length,
        config_.regulated_forward_trust_half_width);
    const double clearance_scale = ClearanceSpeedScale(margin, forward_trusted, config_);
    if (clearance_scale < 1.) {
      linear = std::max(config_.regulated_min_speed, linear * clearance_scale);
    }
    return Twist2d{linear, std::clamp(linear * curvature, -config_.max_angular_velocity,
                                      config_.max_angular_velocity)};
  };
  const double dv = config_.max_linear_acceleration * config_.control_period;
  const double dw = config_.max_angular_acceleration * config_.control_period;
  const auto approach_zero = [](double value, double delta) {
    return std::clamp(0., value - delta, value + delta);
  };
  const auto forward_probe_safe = [&](double heading) {
    const Pose2d aligned = MakePose2d(X(pose), Y(pose), heading);
    return !CollisionImminent(aligned, {config_.regulated_min_speed, 0.}, costmap);
  };
  // Select a heading which advances along the committed route and is
  // executable as a short straight probe after an in-place rotation.  The
  // preferred heading is the first materially different outgoing tangent,
  // which handles Theta* vertices without cutting the inside of a shelf
  // corner. Small symmetric offsets give an off-centre robot a way to move
  // away from the obstacle before pure pursuit rejoins the centreline.
  const auto select_recovery_heading = [&](double* selected) {
    const auto incoming = path[nearest + 1].translation() - path[nearest].translation();
    double preferred = std::atan2(incoming.y(), incoming.x());
    double forward_arc = 0.;
    for (std::size_t index = nearest + 1; index + 1 < path.size(); ++index) {
      const auto segment = path[index + 1].translation() - path[index].translation();
      const double length = segment.norm();
      if (length <= 1e-9) continue;
      forward_arc += length;
      const double candidate = std::atan2(segment.y(), segment.x());
      if (std::abs(Angle(candidate - preferred)) > .10) {
        preferred = candidate;
        break;
      }
      if (forward_arc > config_.max_lookahead_distance) break;
    }
    constexpr std::array<double, 13> kHeadingOffsets{
        0., .10, -.10, .20, -.20, .35, -.35, .50, -.50, .70, -.70, .95, -.95};
    for (const double offset : kHeadingOffsets) {
      const double candidate = Angle(preferred + offset);
      if (forward_probe_safe(candidate)) {
        *selected = candidate;
        return true;
      }
    }
    return false;
  };
  const auto begin_path_alignment = [&]() {
    double heading = 0.;
    if (!select_recovery_heading(&heading)) {
      mode_ = PursuitMode::kRouteInfeasible;
      maneuver_ = ControllerManeuver::kRouteInfeasible;
      fallback_level_ = 7;
      return false;
    }
    recovery_heading_ = heading;
    mode_ = PursuitMode::kStopping;
    stopped_cycles_ = 0;
    aligned_cycles_ = 0;
    return true;
  };
  const auto stopping_command = [&]() {
    maneuver_ = ControllerManeuver::kStopping;
    intentional_stop_ = true;
    fallback_level_ = 5;
    Twist2d braking{approach_zero(current.linear, dv),
                    approach_zero(current.angular, dw)};
    // Prefer physical deceleration. If even that short rollout enters an
    // obstacle, safety takes precedence and the velocity command is zeroed
    // immediately; the simulator/base driver then performs the hard stop.
    if (CollisionImminent(pose, braking, costmap)) braking = {};
    return braking;
  };

  if (mode_ == PursuitMode::kRouteInfeasible) {
    maneuver_ = ControllerManeuver::kRouteInfeasible;
    fallback_level_ = 7;
    return {};
  }
  if (mode_ == PursuitMode::kStopping) {
    const bool stopped = std::abs(current.linear) < .015 &&
                         std::abs(current.angular) < .05;
    stopped_cycles_ = stopped ? stopped_cycles_ + 1 : 0;
    if (stopped_cycles_ < 2) return stopping_command();
    mode_ = PursuitMode::kRotateToPath;
    aligned_cycles_ = 0;
  }
  if (mode_ == PursuitMode::kRotateToPath) {
    maneuver_ = ControllerManeuver::kRotateToPath;
    fallback_level_ = 6;
    double heading_error = Angle(recovery_heading_ - Yaw(pose));
    const bool aligned = std::abs(heading_error) < .08;
    aligned_cycles_ = aligned ? aligned_cycles_ + 1 : 0;
    if (aligned_cycles_ >= 2) {
      // Revalidate the constructive forward probe because the local costmap
      // may have changed while braking and rotating.
      if (!forward_probe_safe(recovery_heading_)) {
        if (!begin_path_alignment()) return {};
        return stopping_command();
      }
      mode_ = PursuitMode::kTracking;
      maneuver_ = ControllerManeuver::kTracking;
      fallback_level_ = 0;
      aligned_cycles_ = 0;
    } else {
      const double desired = std::clamp(2.2 * heading_error,
                                        -config_.max_angular_velocity,
                                        config_.max_angular_velocity);
      // Rotation shim semantics: translation is exactly zero. Applying the
      // linear acceleration clamp here would preserve residual forward speed
      // and reproduce the stage-4 corner collision this state resolves.
      return {0., std::clamp(desired, current.angular - dw, current.angular + dw)};
    }
  }
  // A globally valid polyline may require a tighter turn than its nominal
  // lookahead circle.  Search progressively shorter *path-relative* carrots
  // before declaring a stop.  This is a feasibility projection, not a speed
  // parameter: every candidate remains on the same footprint-checked route.
  double lookahead = nominal_lookahead;
  Twist2d target = target_for(carrot_at(lookahead));
  for (int attempt = 0; attempt < 4 && CollisionImminent(pose, target, costmap); ++attempt) {
    lookahead *= .5;
    target = target_for(carrot_at(lookahead));
  }
  if (CollisionImminent(pose, target, costmap)) {
    if (!begin_path_alignment()) return {};
    return stopping_command();
  }
  // Remember whether the geometric target was rotate-only before the accel
  // clamp reintroduces residual translation from the previous command.
  const bool rotate_only_target = std::abs(target.linear) < 1e-9;
  target.linear = std::clamp(target.linear, current.linear - dv, current.linear + dv);
  target.angular = std::clamp(target.angular, current.angular - dw, current.angular + dw);
  if (CollisionImminent(pose, target, costmap)) {
    // A pure-rotation request that only collides after the linear accel clamp
    // must stop and align — not binary-search a residual forward crawl into
    // the obstacle (shelf-corner / doorway yaw mismatch).
    if (rotate_only_target) {
      if (!begin_path_alignment()) return {};
      return stopping_command();
    }
    // Preserve the executable prefix: shorten the current control arc before
    // abandoning the route.  A binary search gives the largest safe command
    // without rejecting a path merely because its nominal horizon reaches a
    // corner.  This is local (eight collision checks), not a full-route scan.
    Twist2d safe{};
    double low = 0., high = 1.;
    for (int attempt = 0; attempt < 8; ++attempt) {
      const double scale = .5 * (low + high);
      const Twist2d candidate{target.linear * scale, target.angular * scale};
      if (CollisionImminent(pose, candidate, costmap)) high = scale;
      else { low = scale; safe = candidate; }
    }
    if (low > 1e-3) return safe;
    if (!begin_path_alignment()) return {};
    return stopping_command();
  }
  return target;
}

bool RegulatedPurePursuit::CollisionImminent(const Pose2d& pose, Twist2d command,
                                             const LayeredCostmap& costmap) const {
  // The controller uses the same circular footprint as the global planner.
  // Pure rotation is not exempt: if the current footprint is already in
  // contact (for example after a discretized corner cut), publishing another
  // rotation would only spin the wheels against the obstacle.
  const double collision_radius = config_.robot_radius;
  Pose2d projected = pose;
  for (double t = 0.; t <= config_.collision_horizon; t += config_.control_period) {
    const double yaw = Angle(Yaw(projected) + command.angular * config_.control_period);
    projected = MakePose2d(X(projected) + command.linear * std::cos(yaw) * config_.control_period,
                           Y(projected) + command.linear * std::sin(yaw) * config_.control_period,
                           yaw);
    if (costmap.lethal(X(projected), Y(projected), collision_radius)) return true;
  }
  return false;
}

}  // namespace navigation2d
