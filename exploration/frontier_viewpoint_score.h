#pragma once
#include <algorithm>
#include <cmath>

#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::frontier_internal {
// Ranks observation poses inside one frontier cluster (which standoff cell to
// stand on). Tour order between rooms is decided separately by BuildTour with
// nearest-first coverage — do not use this score to jump across the map.
// Prefer standing near the preferred standoff: the old −2·standoff term rewarded
// wall-hugging and sent the robot into side-clearance thrash mid-room.
// Clearance pulls doorway/standoff goals off the jamb toward the free center
// (UI: GOAL glued to the left threshold wall → stuck at the mouth).
inline double ViewpointScore(double information_gain, double travel, double standoff,
                             double gain_weight, double clearance_m = .20) {
  return gain_weight * information_gain / (0.45 + travel) -
         2.5 * std::abs(standoff - navigation2d::kPreferredStandoff()) +
         4.0 * std::min(clearance_m, .40);
}

}  // namespace navigation2d::frontier_internal
