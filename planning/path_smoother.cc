#include "navigation2d/planning/path_smoother.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "navigation2d/planning/collision_checker.h"

namespace navigation2d::planning_internal {
namespace {
double Curvature(const Eigen::Vector2d& first, const Eigen::Vector2d& middle,
                 const Eigen::Vector2d& last) {
  const Eigen::Vector2d a = middle - first, b = last - middle;
  const double an = a.norm(), bn = b.norm(), chord = (last - first).norm();
  if (an < 1e-8 || bn < 1e-8 || chord < 1e-8) return 0.;
  return 2. * std::abs(a.x() * b.y() - a.y() * b.x()) / (an * bn * chord);
}

// Sharp turns are still marked, but wall-foot corners (clearance below the
// soft target) must remain movable — otherwise Theta* corner-cut waypoints
// stay glued to the jamb and the carrot rides the wall.
std::vector<bool> FindAnchors(const Path& path, const DistanceField& distance_field,
                              double min_clearance) {
  std::vector<bool> anchors(path.size(), false);
  if (path.empty()) return anchors;
  anchors.front() = true;
  anchors.back() = true;
  for (std::size_t index = 1; index + 1 < path.size(); ++index) {
    const Eigen::Vector2d before = path[index].translation() - path[index - 1].translation();
    const Eigen::Vector2d after = path[index + 1].translation() - path[index].translation();
    if (before.norm() < 1e-7 || after.norm() < 1e-7 || before.dot(after) < 0.) {
      const double clearance =
          distance_field.distance(path[index].translation().x(),
                                  path[index].translation().y());
      if (clearance + 1e-6 >= .85 * min_clearance) {
        anchors[index] = true;
        if (before.norm() < 1e-7) anchors[index - 1] = true;
        if (after.norm() < 1e-7) anchors[index + 1] = true;
      }
    }
  }
  return anchors;
}

bool SegmentCircleFree(const DistanceField& distance_field, const Eigen::Vector2d& first,
                       const Eigen::Vector2d& second, double robot_radius,
                       double resolution) {
  const int samples = std::max(
      1, static_cast<int>(std::ceil((second - first).norm() / (.5 * resolution))));
  for (int sample = 1; sample <= samples; ++sample) {
    const Eigen::Vector2d point =
        first + static_cast<double>(sample) / samples * (second - first);
    if (!distance_field.CircleCollisionFree(point.x(), point.y(), robot_radius))
      return false;
  }
  return true;
}

// Shift waypoints laterally toward the free-space medial line so doorway /
// corridor exits go through the middle then turn, instead of hugging the wall
// foot (loc delay makes wall-foot carrots collide).
void CenterInClearance(std::vector<Eigen::Vector2d>* points,
                       const DistanceField& distance_field, double robot_radius,
                       double target_clearance, double max_shift,
                       double resolution) {
  if (points == nullptr || points->size() < 3) return;
  auto& pts = *points;
  for (std::size_t index = 1; index + 1 < pts.size(); ++index) {
    Eigen::Vector2d tangent = pts[index + 1] - pts[index - 1];
    if (tangent.norm() < 1e-6) tangent = pts[index + 1] - pts[index];
    if (tangent.norm() < 1e-6) continue;
    tangent.normalize();
    const Eigen::Vector2d normal(-tangent.y(), tangent.x());
    const double current =
        distance_field.distance(pts[index].x(), pts[index].y());
    Eigen::Vector2d best = pts[index];
    double best_clearance = current;
    // Always maximize local clearance (door/corridor medial line). Stopping at
    // target left entry paths parked on the wall foot once clearance hit 0.16.
    const double shift_budget =
        current >= target_clearance ? .5 * max_shift : max_shift;
    const int steps = std::max(
        2, static_cast<int>(std::ceil(shift_budget / std::max(.02, .5 * resolution))));
    for (int step = -steps; step <= steps; ++step) {
      if (step == 0) continue;
      const double offset = shift_budget * static_cast<double>(step) / steps;
      const Eigen::Vector2d candidate = pts[index] + offset * normal;
      if (!distance_field.CircleCollisionFree(candidate.x(), candidate.y(),
                                              robot_radius))
        continue;
      if (!SegmentCircleFree(distance_field, pts[index - 1], candidate,
                             robot_radius, resolution) ||
          !SegmentCircleFree(distance_field, candidate, pts[index + 1],
                             robot_radius, resolution))
        continue;
      const double clearance =
          distance_field.distance(candidate.x(), candidate.y());
      // Prefer higher clearance; once above target, small gains still win so
      // the path sits in the free middle before turning.
      if (clearance > best_clearance + 1e-4) {
        best_clearance = clearance;
        best = candidate;
      }
    }
    pts[index] = best;
  }
}
}  // namespace

Path ConstrainedPathSmoother::Smooth(const Path& path, const LayeredCostmap& costmap,
                                     double robot_radius) const {
  if (!config_.smoother_enabled || path.size() < 3) return path;
  const DistanceField distance_field(costmap);
  const double resolution = costmap.grid().resolution();
  const auto anchors =
      FindAnchors(path, distance_field, config_.smoother_min_clearance);
  std::vector<Eigen::Vector2d> original, points;
  original.reserve(path.size());
  for (const auto& pose : path) original.push_back(pose.translation());
  points = original;
  for (int iteration = 0; iteration < config_.smoother_max_iterations; ++iteration) {
    double largest_change = 0.;
    for (std::size_t index = 1; index + 1 < points.size(); ++index) {
      const double clearance =
          distance_field.distance(points[index].x(), points[index].y());
      Eigen::Vector2d update = Eigen::Vector2d::Zero();
      if (!anchors[index]) {
        update = config_.smoother_data_weight * (original[index] - points[index]) +
            config_.smoother_smooth_weight *
                (points[index - 1] + points[index + 1] - 2. * points[index]);
      }
      // Soft push off obstacles for every movable point — including turn
      // corners that would otherwise stay on the Theta* wall chord.
      if (clearance < config_.smoother_min_clearance) {
        const Eigen::Vector2d gradient =
            distance_field.gradient(points[index].x(), points[index].y());
        if (gradient.norm() > 1e-9)
          update += config_.smoother_obstacle_weight *
              (config_.smoother_min_clearance - clearance) * gradient.normalized();
      } else if (anchors[index]) {
        continue;
      }
      if (update.norm() < 1e-12) continue;
      if (update.norm() > config_.smoother_max_step)
        update *= config_.smoother_max_step / update.norm();
      Eigen::Vector2d candidate = points[index] + update;
      const Eigen::Vector2d deviation = candidate - original[index];
      if (deviation.norm() > config_.smoother_max_deviation)
        candidate = original[index] + deviation.normalized() * config_.smoother_max_deviation;
      bool curvature_valid = Curvature(points[index - 1], candidate, points[index + 1]) <=
                             config_.smoother_max_curvature;
      if (index >= 2)
        curvature_valid = curvature_valid && Curvature(
            points[index - 2], points[index - 1], candidate) <= config_.smoother_max_curvature;
      if (index + 2 < points.size())
        curvature_valid = curvature_valid && Curvature(
            candidate, points[index + 1], points[index + 2]) <= config_.smoother_max_curvature;
      if (!curvature_valid ||
          !SegmentCircleFree(distance_field, points[index - 1], candidate, robot_radius,
                             resolution) ||
          !SegmentCircleFree(distance_field, candidate, points[index + 1], robot_radius,
                             resolution))
        continue;
      largest_change = std::max(largest_change, (candidate - points[index]).norm());
      points[index] = candidate;
    }
    if (largest_change < config_.smoother_tolerance) break;
  }
  // Explicit medial shift: exit doorways / corridors through the free middle
  // then turn, instead of cutting the inside corner along the wall foot.
  CenterInClearance(&points, distance_field, robot_radius,
                    config_.smoother_min_clearance,
                    std::max(config_.smoother_max_deviation, .20), resolution);

  Path result = path;
  for (std::size_t index = 0; index < result.size(); ++index) {
    double yaw = Yaw(path[index]);
    if (index + 1 < points.size()) {
      const Eigen::Vector2d tangent = points[index + 1] - points[index];
      if (tangent.norm() > 1e-9) yaw = std::atan2(tangent.y(), tangent.x());
    }
    result[index] = MakePose2d(points[index].x(), points[index].y(), yaw);
  }
  return result;
}

}  // namespace navigation2d::planning_internal
