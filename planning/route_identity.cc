#include "navigation2d/planning/route_identity.h"

#include <cmath>

namespace navigation2d::planning_internal {
namespace {
std::uint64_t MixSignature(std::uint64_t hash, std::uint64_t value) {
  hash ^= value + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
  return hash;
}

}  // namespace

std::uint64_t FirstSegmentSignature(const Path& path, const Pose2d& pose, double resolution) {
  if (resolution <= 1e-9) {
    resolution = .05;
  }
  int yaw_bin = static_cast<int>(std::lround(Yaw(pose) * 16. / (2. * std::acos(-1.)))) % 16;
  if (yaw_bin < 0) {
    yaw_bin += 16;
  }
  std::uint64_t hash = 0;
  hash = MixSignature(hash, static_cast<std::uint64_t>(
                                static_cast<int>(std::floor(X(pose) / resolution)) + 100000));
  hash = MixSignature(hash, static_cast<std::uint64_t>(
                                static_cast<int>(std::floor(Y(pose) / resolution)) + 100000));
  hash = MixSignature(hash, static_cast<std::uint64_t>(yaw_bin));
  double arc = 0.;
  int edges = 0;
  for (std::size_t index = 1; index < path.size() && edges < 3 && arc < 1.; ++index) {
    const auto delta = path[index].translation() - path[index - 1].translation();
    const double length = delta.norm();
    if (length <= 1e-4) {
      int yaw_delta = static_cast<int>(
          std::lround(std::atan2(std::sin(Yaw(path[index]) - Yaw(path[index - 1])),
                                 std::cos(Yaw(path[index]) - Yaw(path[index - 1]))) *
                      16. / (2. * std::acos(-1.))));
      hash = MixSignature(hash, 0x1000ULL + static_cast<std::uint64_t>(yaw_delta + 32));
      ++edges;
      continue;
    }
    hash =
        MixSignature(hash, static_cast<std::uint64_t>(
                               static_cast<int>(std::floor(X(path[index]) / resolution)) + 200000));
    hash =
        MixSignature(hash, static_cast<std::uint64_t>(
                               static_cast<int>(std::floor(Y(path[index]) / resolution)) + 200000));
    arc += length;
    ++edges;
  }
  return hash;
}
}  // namespace navigation2d::planning_internal
