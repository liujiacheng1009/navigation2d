#pragma once

#include "navigation2/types.h"

namespace navigation2d {

class RegulatedPurePursuit {
 public:
  Velocity Compute(const Path& path, const Pose2d& pose, double speed) const;
};

}  // namespace navigation2d
