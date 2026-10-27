#include <algorithm>
#include <cmath>

#include "navigation2d/planning/path_geometry.h"
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

Path DensifyBodyHeadingRoute(const Path& sparse, double maximum_step) {
  if (sparse.size() < 2) {
    return sparse;
  }
  Path dense;
  dense.reserve(sparse.size());
  dense.push_back(sparse.front());
  for (std::size_t i = 1; i < sparse.size(); ++i) {
    const auto start = sparse[i - 1].translation();
    const auto segment = sparse[i].translation() - start;
    const double yaw = Yaw(sparse[i - 1]);
    const double difference = Yaw(sparse[i]) - yaw;
    const double turn = std::atan2(std::sin(difference), std::cos(difference));
    const int steps = std::max(1, static_cast<int>(std::ceil(segment.norm() / maximum_step)));
    for (int step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / steps;
      const auto point = start + ratio * segment;
      // Keep original endpoints exactly, including zero-length rotation/cusps.
      dense.push_back(step == steps ? sparse[i]
                                    : MakePose2d(point.x(), point.y(), yaw + ratio * turn));
    }
  }
  return dense;
}
}  // namespace navigation2d::planning_internal
