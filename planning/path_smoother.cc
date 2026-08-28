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

std::vector<bool> FindAnchors(const Path& path) {
  std::vector<bool> anchors(path.size(), false);
  if (path.empty()) return anchors;
  anchors.front() = true;
  anchors.back() = true;
  for (std::size_t index = 1; index + 1 < path.size(); ++index) {
    const Eigen::Vector2d before = path[index].translation() - path[index - 1].translation();
    const Eigen::Vector2d after = path[index + 1].translation() - path[index].translation();
    if (before.norm() < 1e-7 || after.norm() < 1e-7 || before.dot(after) < 0.) {
      anchors[index] = true;
      if (before.norm() < 1e-7) anchors[index - 1] = true;
      if (after.norm() < 1e-7) anchors[index + 1] = true;
    }
  }
  return anchors;
}
}  // namespace

Path ConstrainedPathSmoother::Smooth(const Path& path, const LayeredCostmap& costmap,
                                     double robot_radius) const {
  if (!config_.smoother_enabled || path.size() < 3) return path;
  const DistanceField distance_field(costmap);
  const auto anchors = FindAnchors(path);
  std::vector<Eigen::Vector2d> original, points;
  original.reserve(path.size());
  for (const auto& pose : path) original.push_back(pose.translation());
  points = original;
  for (int iteration = 0; iteration < config_.smoother_max_iterations; ++iteration) {
    double largest_change = 0.;
    for (std::size_t index = 1; index + 1 < points.size(); ++index) {
      if (anchors[index]) continue;
      Eigen::Vector2d update = config_.smoother_data_weight * (original[index] - points[index]) +
          config_.smoother_smooth_weight * (points[index - 1] + points[index + 1] - 2. * points[index]);
      const double clearance = distance_field.distance(points[index].x(), points[index].y());
      if (clearance < config_.smoother_min_clearance) {
        const Eigen::Vector2d gradient = distance_field.gradient(points[index].x(), points[index].y());
        if (gradient.norm() > 1e-9)
          update += config_.smoother_obstacle_weight *
              (config_.smoother_min_clearance - clearance) * gradient.normalized();
      }
      if (update.norm() > config_.smoother_max_step)
        update *= config_.smoother_max_step / update.norm();
      Eigen::Vector2d candidate = points[index] + update;
      const Eigen::Vector2d deviation = candidate - original[index];
      if (deviation.norm() > config_.smoother_max_deviation)
        candidate = original[index] + deviation.normalized() * config_.smoother_max_deviation;
      const auto segment_free = [&](const Eigen::Vector2d& first, const Eigen::Vector2d& second) {
        const int samples = std::max(1, static_cast<int>(std::ceil(
            (second - first).norm() / (.5 * costmap.grid().resolution()))));
        for (int sample = 1; sample <= samples; ++sample) {
          const Eigen::Vector2d point = first + static_cast<double>(sample) / samples * (second - first);
          if (!distance_field.CircleCollisionFree(point.x(), point.y(), robot_radius)) return false;
        }
        return true;
      };
      bool curvature_valid = Curvature(points[index - 1], candidate, points[index + 1]) <=
                             config_.smoother_max_curvature;
      if (index >= 2)
        curvature_valid = curvature_valid && Curvature(
            points[index - 2], points[index - 1], candidate) <= config_.smoother_max_curvature;
      if (index + 2 < points.size())
        curvature_valid = curvature_valid && Curvature(
            candidate, points[index + 1], points[index + 2]) <= config_.smoother_max_curvature;
      if (!curvature_valid || !segment_free(points[index - 1], candidate) ||
          !segment_free(candidate, points[index + 1])) continue;
      largest_change = std::max(largest_change, (candidate - points[index]).norm());
      points[index] = candidate;
    }
    if (largest_change < config_.smoother_tolerance) break;
  }
  Path result = path;
  for (std::size_t index = 0; index < result.size(); ++index) {
    double yaw = Yaw(path[index]);
    if (!anchors[index] && index + 1 < points.size()) {
      const Eigen::Vector2d tangent = points[index + 1] - points[index];
      if (tangent.norm() > 1e-9) yaw = std::atan2(tangent.y(), tangent.x());
    }
    result[index] = MakePose2d(points[index].x(), points[index].y(), yaw);
  }
  return result;
}

}  // namespace navigation2d::planning_internal
