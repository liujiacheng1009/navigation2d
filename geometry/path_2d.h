#pragma once

#include <vector>

#include "navigation2d/geometry/pose_2d.h"

namespace navigation2d {

// Ordered route in the caller-declared map frame; poses use metres/radians.
using Path = std::vector<Pose2d>;

}  // namespace navigation2d
