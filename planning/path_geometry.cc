#include "navigation2d/planning/path_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation2d::planning_internal {
Path DensifyRoute(const Path& sparse, double maximum_step) {
  if (sparse.size() < 2) {
    return sparse;
  }
  Path dense;
  dense.reserve(sparse.size());
  dense.push_back(sparse.front());
  for (std::size_t index = 1; index < sparse.size(); ++index) {
    const auto start = sparse[index - 1].translation();
    const auto segment = sparse[index].translation() - start;
    const double length = segment.norm();
    if (length <= 1e-9) {
      continue;
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(length / maximum_step)));
    const double yaw = std::atan2(segment.y(), segment.x());
    for (int step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / steps;
      const auto point = start + ratio * segment;
      dense.push_back(MakePose2d(point.x(), point.y(), yaw));
    }
  }
  if (!dense.empty()) {
    dense.back() = MakePose2d(X(sparse.back()), Y(sparse.back()), Yaw(sparse.back()));
  }
  return dense;
}

bool PathFootprintValid(const Path& path, const LayeredCostmap& costmap, double robot_radius) {
  if (path.empty()) {
    return false;
  }
  const double sample_step = std::max(.02, .5 * costmap.grid().resolution());
  for (std::size_t index = 0; index < path.size(); ++index) {
    if (costmap.lethal(X(path[index]), Y(path[index]), robot_radius)) {
      return false;
    }
    if (index == 0) {
      continue;
    }
    const Eigen::Vector2d start = path[index - 1].translation();
    const Eigen::Vector2d segment = path[index].translation() - start;
    const double length = segment.norm();
    if (length <= 1e-9) {
      continue;
    }
    const int samples = std::max(1, static_cast<int>(std::ceil(length / sample_step)));
    for (int sample = 1; sample <= samples; ++sample) {
      const Eigen::Vector2d point = start + (static_cast<double>(sample) / samples) * segment;
      if (costmap.lethal(point.x(), point.y(), robot_radius)) {
        return false;
      }
    }
  }
  return true;
}

// Project onto the *ordered* route, never onto an unconstrained cloud of
// waypoints.  The returned coordinate is arc length from the path start.
//
// The small forward window is intentional: a path may pass close to itself
// in a warehouse aisle.  A global closest-point query can then teleport the
// progress state across a different branch and incorrectly declare motion.
double PathProgress(const Path& path, const Pose2d& pose, double minimum_progress) {
  if (path.size() < 2) {
    return 0.;
  }
  constexpr double kBackwardSlack = .15;
  constexpr double kForwardWindow = 1.25;
  double arc = 0., best_arc = std::max(0., minimum_progress);
  double best_distance = std::numeric_limits<double>::infinity();
  const double begin = std::max(0., minimum_progress - kBackwardSlack);
  const double end = minimum_progress + kForwardWindow;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto& first = path[i - 1].translation();
    const auto& second = path[i].translation();
    const auto segment = second - first;
    const double length = segment.norm();
    if (length <= 1e-9) {
      continue;
    }
    const double segment_end = arc + length;
    if (segment_end < begin) {
      arc = segment_end;
      continue;
    }
    if (arc > end) {
      break;
    }
    const double t =
        std::clamp((pose.translation() - first).dot(segment) / (length * length), 0., 1.);
    const double projected_arc = arc + t * length;
    if (projected_arc >= begin && projected_arc <= end) {
      const double distance = (pose.translation() - (first + t * segment)).norm();
      if (distance < best_distance) {
        best_distance = distance;
        best_arc = projected_arc;
      }
    }
    arc = segment_end;
  }
  return best_arc;
}
double DistanceToPath(const Path& path, const Pose2d& pose) {
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1; index < path.size(); ++index) {
    const auto start = path[index - 1].translation();
    const auto segment = path[index].translation() - start;
    const double squared = segment.squaredNorm();
    if (squared <= 1e-12) {
      continue;
    }
    const double ratio = std::clamp((pose.translation() - start).dot(segment) / squared, 0., 1.);
    best = std::min(best, (pose.translation() - (start + ratio * segment)).norm());
  }
  return std::isfinite(best) ? best : 0.;
}
}  // namespace navigation2d::planning_internal
