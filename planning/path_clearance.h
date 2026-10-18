#pragma once

#include <limits>

#include "navigation2d/costmap/layered_costmap.h"
#include "navigation2d/geometry/robot_geometry.h"
#include "navigation2d/planning/global_planner.h"

namespace navigation2d {

// Segmented clearance along a densified GLOBAL path. Distinguishes wall-foot
// chords in open rooms from legitimate door throats via perpendicular
// free_width (left+right ray to lethal/unknown), not min(margin) alone.
struct PathClearanceProfile {
  double path_min_clearance_m = std::numeric_limits<double>::infinity();
  double open_scrape_length_m = 0.;
  double bottleneck_min_clearance_m = std::numeric_limits<double>::infinity();
  bool has_open_wall_scrape = false;
  bool has_lethal_tip = false;
};

PathClearanceProfile AnalyzePathClearanceProfile(const Path& path,
                                                 const LayeredCostmap& costmap,
                                                 double robot_radius);

inline bool PathClearanceProfileAcceptable(const PathClearanceProfile& profile) {
  return !profile.has_open_wall_scrape && !profile.has_lethal_tip;
}

inline bool PathClearanceProfileAcceptable(
    const GlobalPlanningDiagnostics& diagnostics) {
  return !diagnostics.has_open_wall_scrape && !diagnostics.has_lethal_tip;
}

}  // namespace navigation2d
