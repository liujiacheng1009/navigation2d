#pragma once
#include <cstdint>
namespace navigation2d::exploration {
// Planar pose in the mission's map frame (metres/radians). The original
// message stamp is retained as nanoseconds; freshness policy is unchanged.
struct PoseObservation {
  double x = 0., y = 0., yaw = 0.;
  std::int64_t stamp_ns = 0;
};
}  // namespace navigation2d::exploration
