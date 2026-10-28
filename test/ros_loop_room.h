#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace navigation2d::test {
enum class RosLoopScenario { kReturn, kFrontier, kObstacle, kScanPause };

// Immutable physical world for synthetic sensors and collision checks. The
// same rectangle boundaries define occupied cells and analytic ray returns.
class RosLoopRoom {
 public:
  explicit RosLoopRoom(bool obstacle = false) : obstacle_(obstacle) {}
  static constexpr int kWidth = 200, kHeight = 150;
  static constexpr double kResolution = .02;
  std::int8_t Cell(int x, int y, bool revealed) const {
    if (x <= 0 || y <= 0 || x >= 199 || y >= 149) {
      return 100;
    }
    if (obstacle_ && x >= 80 && x <= 92 && y >= 45 && y <= 104) {
      return 100;
    }
    return !revealed && x > 137 ? -1 : 0;
  }
  double Range(double x, double y, double angle) const {
    const double c = std::cos(angle), s = std::sin(angle);
    double range = 8.;
    if (std::abs(c) > 1e-9) {
      range = std::min(range, (c > 0. ? 3.98 - x : .02 - x) / c);
    }
    if (std::abs(s) > 1e-9) {
      range = std::min(range, (s > 0. ? 2.98 - y : .02 - y) / s);
    }
    if (obstacle_) {
      double near = 0., far = std::numeric_limits<double>::infinity();
      if (Clip(x, c, 1.6, 1.86, near, far) && Clip(y, s, .9, 2.1, near, far)) {
        range = std::min(range, near);
      }
    }
    return range;
  }
  // Conservative circular envelope supplied by the caller's chassis profile.
  bool HitsObstacle(double x, double y, double radius) const {
    if (!obstacle_) {
      return false;
    }
    const double dx = std::max({1.6 - x, 0., x - 1.86});
    const double dy = std::max({.9 - y, 0., y - 2.1});
    return std::hypot(dx, dy) <= radius;
  }

 private:
  static bool Clip(double origin, double direction, double low, double high, double& near,
                   double& far) {
    if (std::abs(direction) < 1e-12) {
      return origin >= low && origin <= high;
    }
    const double first = (low - origin) / direction;
    const double last = (high - origin) / direction;
    near = std::max(near, std::min(first, last));
    far = std::min(far, std::max(first, last));
    return far >= near;
  }
  const bool obstacle_;
};
}  // namespace navigation2d::test
