#include "navigation2/control/regulated_pure_pursuit.h"

#include <algorithm>
#include <cmath>

namespace navigation2d {
namespace { double Angle(double a) { return std::atan2(std::sin(a), std::cos(a)); } }

Velocity RegulatedPurePursuit::Compute(const Path& path, const Pose2d& pose, double speed) const {
  size_t nearest = 0;
  double nearest_distance = 1e30;
  for (size_t i = 0; i < path.size(); ++i) {
    const double d = std::hypot(path[i].x - pose.x, path[i].y - pose.y);
    if (d < nearest_distance) { nearest_distance = d; nearest = i; }
  }
  const double lookahead = std::clamp(std::abs(speed) * 1.2, .18, .42);
  size_t carrot = nearest;
  double accumulated = 0.;
  while (carrot + 1 < path.size() && accumulated < lookahead) {
    accumulated += std::hypot(path[carrot + 1].x - path[carrot].x,
                              path[carrot + 1].y - path[carrot].y);
    ++carrot;
  }
  const double dx = path[carrot].x - pose.x, dy = path[carrot].y - pose.y;
  const double heading_error = Angle(std::atan2(dy, dx) - pose.yaw);
  if (std::abs(heading_error) > .40) return {0., std::clamp(2.2 * heading_error, -1.8, 1.8)};
  const double distance_to_goal = std::hypot(path.back().x - pose.x, path.back().y - pose.y);
  double linear = std::min(.5, std::max(.06, distance_to_goal));
  linear *= std::max(.35, 1. - std::abs(heading_error));
  const double lateral = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
  const double curvature = 2. * lateral / std::max(.04, dx * dx + dy * dy);
  return {linear, std::clamp(linear * curvature, -1.8, 1.8)};
}

}  // namespace navigation2d
