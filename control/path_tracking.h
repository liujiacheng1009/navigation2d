#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>

#include "navigation2d/planning/global_planner.h"

namespace navigation2d::control_internal {

// A controller follows a path monotonically in normal operation.  Retaining
// that progress makes nearest-point lookup independent of global path length.
struct PathSearchState {
  const Pose2d* data = nullptr;
  std::size_t size = 0;
  std::size_t nearest = 0;
  std::size_t last_evaluations = 0;
};

namespace path_tracking_internal {

inline constexpr std::size_t kCoarseSamples = 128;
inline constexpr std::size_t kBackwardWindow = 64;
inline constexpr std::size_t kForwardWindow = 192;
inline constexpr double kPathLostDistanceSquared = 4.;

inline std::size_t ScanRange(const Path& path, const Pose2d& pose,
                             std::size_t first, std::size_t last,
                             std::size_t stride, std::size_t* evaluations) {
  std::size_t nearest = first;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t index = first; index <= last;) {
    const double distance =
        (path[index].translation() - pose.translation()).squaredNorm();
    ++*evaluations;
    if (distance < best) {
      best = distance;
      nearest = index;
    }
    if (last - index < stride) break;
    index += stride;
  }
  if ((last - first) % stride != 0) {
    const double distance =
        (path[last].translation() - pose.translation()).squaredNorm();
    ++*evaluations;
    if (distance < best) nearest = last;
  }
  return nearest;
}

// Recursively narrow the sampled interval. This makes recovery logarithmic in
// path length instead of scanning the whole interval between two samples.
inline std::size_t CoarseToLocal(const Path& path, const Pose2d& pose,
                                 std::size_t* evaluations) {
  std::size_t first = 0;
  std::size_t last = path.size() - 1;
  while (last - first + 1 > kBackwardWindow + kForwardWindow + 1) {
    const std::size_t stride = std::max<std::size_t>(
        1, (last - first + kCoarseSamples - 1) / kCoarseSamples);
    const std::size_t coarse = ScanRange(path, pose, first, last, stride, evaluations);
    first = coarse > stride ? std::max(first, coarse - stride) : first;
    last = std::min(last, coarse + stride);
  }
  return ScanRange(path, pose, first, last, 1, evaluations);
}

}  // namespace path_tracking_internal

// Bounded lookup: a new path is sampled uniformly and refined locally; an
// unchanged path is searched only around the previous progress point.
inline std::size_t FindNearestPathPoint(const Path& path, const Pose2d& pose,
                                        PathSearchState* state) {
  using namespace path_tracking_internal;
  if (path.empty()) return 0;
  const bool same_path = state->data == path.data() && state->size == path.size();
  std::size_t evaluations = 0;
  std::size_t nearest = 0;
  if (same_path) {
    const std::size_t first = state->nearest > kBackwardWindow
        ? state->nearest - kBackwardWindow : 0;
    const std::size_t last = std::min(path.size() - 1,
                                      state->nearest + kForwardWindow);
    nearest = ScanRange(path, pose, first, last, 1, &evaluations);
    const double local_distance =
        (path[nearest].translation() - pose.translation()).squaredNorm();
    // A controller may be restarted or the robot may be displaced much farther
    // than its progress window. Recover with the same bounded first-lookup
    // procedure instead of reverting to a full path scan.
    if (local_distance > kPathLostDistanceSquared) {
      nearest = CoarseToLocal(path, pose, &evaluations);
    }
  } else {
    nearest = CoarseToLocal(path, pose, &evaluations);
  }
  state->data = path.data();
  state->size = path.size();
  state->nearest = nearest;
  state->last_evaluations = evaluations;
  return nearest;
}

}  // namespace navigation2d::control_internal
