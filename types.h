#pragma once

#include <vector>

namespace navigation2d {

struct Pose2d { double x = 0.; double y = 0.; double yaw = 0.; };
struct Velocity { double linear = 0.; double angular = 0.; };
using Path = std::vector<Pose2d>;

}  // namespace navigation2d
