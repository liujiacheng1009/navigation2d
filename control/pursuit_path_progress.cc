#include "navigation2d/control/pursuit_path_progress.h"

#include <algorithm>
#include <cassert>
#include <cmath>
namespace navigation2d::pursuit_internal {
bool PathProgress::Observe(const Path& path) {
  assert(!path.empty());
  const bool path_replaced =
      path.size() != path_size_ || path.empty() ||
      std::hypot(X(path.front()) - path_start_x_, Y(path.front()) - path_start_y_) > .05 ||
      std::hypot(X(path.back()) - path_goal_x_, Y(path.back()) - path_goal_y_) > .05;
  if (path_replaced) {
    progress_index_ = 0;
    path_size_ = path.size();
    if (!path.empty()) {
      path_start_x_ = X(path.front());
      path_start_y_ = Y(path.front());
      path_goal_x_ = X(path.back());
      path_goal_y_ = Y(path.back());
    }
  }
  return path_replaced;
}
PathProjection PathProgress::Project(const Path& path, const Pose2d& pose) {
  assert(path.size() >= 2);
  std::size_t nearest = std::min(progress_index_, path.size() - 2);
  double nearest_t = 0.;
  double nearest_distance = 1e30;
  double searched_arc = 0.;
  constexpr double kProjectionWindow = 1.25;
  for (std::size_t i = nearest; i + 1 < path.size(); ++i) {
    const auto start = path[i].translation();
    const auto segment = path[i + 1].translation() - start;
    const double squared_length = segment.squaredNorm();
    if (squared_length <= 1e-12) {
      continue;
    }
    if (searched_arc > kProjectionWindow) {
      break;
    }
    const double t = std::clamp((pose.translation() - start).dot(segment) / squared_length, 0., 1.);
    const double distance = (pose.translation() - (start + t * segment)).norm();
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = i;
      nearest_t = t;
    }
    searched_arc += std::sqrt(squared_length);
  }
  progress_index_ = std::max(progress_index_, nearest);
  return {nearest, nearest_t};
}
Eigen::Vector2d CarrotAt(const Path& path, PathProjection projection, double lookahead) {
  Eigen::Vector2d carrot = path[projection.segment].translation() +
                           projection.t * (path[projection.segment + 1].translation() -
                                           path[projection.segment].translation());
  double remaining = lookahead;
  std::size_t segment_index = projection.segment;
  double segment_t = projection.t;
  while (segment_index + 1 < path.size()) {
    const auto start = path[segment_index].translation();
    const auto end = path[segment_index + 1].translation();
    const auto segment = end - start;
    const double length = segment.norm();
    if (length <= 1e-12) {
      ++segment_index;
      segment_t = 0.;
      continue;
    }
    const double available = (1. - segment_t) * length;
    if (remaining <= available) {
      carrot += (remaining / length) * segment;
      break;
    }
    carrot = end;
    remaining -= available;
    ++segment_index;
    segment_t = 0.;
  }
  return carrot;
}
}  // namespace navigation2d::pursuit_internal
