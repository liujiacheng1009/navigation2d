#pragma once
#include <cstdint>

#include "navigation2d/geometry/path_2d.h"

namespace navigation2d::planning_internal {
// Exact legacy identity: start cell/yaw and first three route edges, up to 1 m.
std::uint64_t FirstSegmentSignature(const Path& path, const Pose2d& pose, double resolution);
}  // namespace navigation2d::planning_internal
