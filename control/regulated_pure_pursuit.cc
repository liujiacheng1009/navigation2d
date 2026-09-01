#include "navigation2d/control/regulated_pure_pursuit.h"

#include <algorithm>
#include <cmath>

namespace navigation2d {
namespace { double Angle(double a) { return std::atan2(std::sin(a), std::cos(a)); } }

Twist2d RegulatedPurePursuit::Compute(const Path& path, const Pose2d& pose, Twist2d current,
                                       const LayeredCostmap& costmap,
                                       const std::vector<PredictedObstacle>& dynamic_obstacles) const {
  (void)dynamic_obstacles;
  fallback_level_ = 0;
  if (path.empty()) return {};
  const bool path_replaced = path.size() != path_size_ || path.empty() ||
      std::hypot(X(path.front()) - path_start_x_, Y(path.front()) - path_start_y_) > .05 ||
      std::hypot(X(path.back()) - path_goal_x_, Y(path.back()) - path_goal_y_) > .05;
  if (path_replaced) {
    progress_index_ = 0;
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
    const auto [cx, cy] = costmap.grid().ToCell(X(pose), Y(pose));
    const double normalized_cost = static_cast<double>(costmap.cost(cx, cy)) / 252.;
    linear *= std::max(.25, 1. - normalized_cost);
    return Twist2d{linear, std::clamp(linear * curvature, -config_.max_angular_velocity,
                                      config_.max_angular_velocity)};
  };
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
    fallback_level_ = 1;
    // A curvature-constrained base cannot enter every globally valid corner
    // from every heading in one continuous arc.  Align with the tangent of
    // the next validated segment, then resume normal pursuit on the next
    // cycle. This bounded geometric manoeuvre replaces the old 18 s
    // zero-command wait and is used only when every forward arc is unsafe.
    const auto incoming = path[nearest + 1].translation() - path[nearest].translation();
    double tangent_heading = std::atan2(incoming.y(), incoming.x());
    double forward_arc = 0.;
    // At a sharp vertex the current (incoming) tangent is already aligned,
    // yet continuing straight is exactly the unsafe command. Find the first
    // materially different outgoing tangent in the local route and align to
    // it. Densification makes this search independent of Theta* vertex gaps.
    for (std::size_t index = nearest + 1; index + 1 < path.size(); ++index) {
      const auto segment = path[index + 1].translation() - path[index].translation();
      const double length = segment.norm();
      if (length <= 1e-9) continue;
      forward_arc += length;
      const double candidate = std::atan2(segment.y(), segment.x());
      if (std::abs(Angle(candidate - tangent_heading)) > .10) {
        tangent_heading = candidate;
        break;
      }
      if (forward_arc > config_.max_lookahead_distance) break;
    }
    const double tangent_error = Angle(tangent_heading - Yaw(pose));
    if (std::abs(tangent_error) > config_.rotate_to_heading_min_angle) {
      fallback_level_ = 2;
      target = {0., std::clamp(2.2 * tangent_error,
                              -config_.max_angular_velocity, config_.max_angular_velocity)};
    } else {
      fallback_level_ = 3;
      // Do not turn an already aligned tangent fallback into a zero-command
      // fixed point. The dense global route has been footprint checked, so a
      // short motion along its exact local tangent is the constructive way
      // out of the degenerate pure-pursuit circle. It also preserves the
      // forward-only contract instead of invoking a scripted retreat.
      target = {config_.regulated_min_speed,
                std::clamp(2.2 * tangent_error,
                           -config_.max_angular_velocity, config_.max_angular_velocity)};
    }
  }
  const double dv = config_.max_linear_acceleration * config_.control_period;
  const double dw = config_.max_angular_acceleration * config_.control_period;
  target.linear = std::clamp(target.linear, current.linear - dv, current.linear + dv);
  target.angular = std::clamp(target.angular, current.angular - dw, current.angular + dw);
  if (CollisionImminent(pose, target, costmap)) {
    fallback_level_ = 4;
    return {0., 0.};
  }
  return target;
}

bool RegulatedPurePursuit::CollisionImminent(const Pose2d& pose, Twist2d command,
                                             const LayeredCostmap& costmap) const {
  Pose2d projected = pose;
  for (double t = 0.; t <= config_.collision_horizon; t += config_.control_period) {
    const double yaw = Angle(Yaw(projected) + command.angular * config_.control_period);
    projected = MakePose2d(X(projected) + command.linear * std::cos(yaw) * config_.control_period,
                           Y(projected) + command.linear * std::sin(yaw) * config_.control_period,
                           yaw);
    if (costmap.lethal(X(projected), Y(projected), config_.robot_radius)) return true;
  }
  return false;
}

}  // namespace navigation2d
