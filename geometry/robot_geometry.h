#pragma once

#include <cmath>

namespace navigation2d {

// Inscribed planning radius for the chassis (CAD max footprint ~0.207 m +
// margin). Explorer nomination, PathCuts, and autonomous explore Navigation
// configs share this value — do not scatter bare 0.22 literals.
inline constexpr double kPlanningRadius = .22;

// One planning-circle diameter. Retreat lengths and near-goal windows use
// this as a travel distance. It is not a lidar stuck threshold: a return
// inside the diameter can still be outside the chassis.
inline constexpr double kSwingClearance = 2. * kPlanningRadius;

// Cone half-angle whose lateral span at kSwingClearance equals one body
// radius: atan(kPlanningRadius / kSwingClearance) = atan(1/2).
// Every forward/side stall sample uses this aperture.
inline const double kBodyConeHalfAngle =
    std::atan(kPlanningRadius / kSwingClearance);

// NavigationSystem::AddPathMetrics reports
//   path_min_clearance_m = dist_to_obstacle - robot_radius
// i.e. EXTRA free margin beyond the body — not the raw obstacle distance.
//
// Open tier wants one extra body radius off obstacles (mid-aisle when the
// aisle is wider than the body). Pinch tier only requires the planning
// circle itself to be clear: any strictly positive extra margin. A fixed
// door width is not part of this contract.
inline constexpr double kOpenPreferPathMargin = kPlanningRadius;
inline constexpr double kPinchMinPathMargin = 0.;

// Trail-reverse distances, in planning-circle diameters. Entry/open leave
// the cell; a pocket retreat is longer because both axes are inside the
// planning circle.
inline constexpr double kEntryRetreat = 2. * kSwingClearance;
inline constexpr double kOpenRetreat = 3. * kSwingClearance;
inline constexpr double kPocketRetreat = 4. * kSwingClearance;

// Lidar range is distance to the hit. 0 means that sector returned nothing.
// Yaw sweeps the planning circle. A return inside that circle is the body
// on that axis; the diameter (0.44 m) is not required. A centered 0.70 m
// door leaves about 0.35 m to each jamb, which is outside this circle.
inline bool HitInsideBody(double range) {
  return range > 0. && range < kPlanningRadius;
}
inline bool HitBlocksSwing(double range) { return HitInsideBody(range); }
inline bool AxisSwingClear(double range) {
  return range >= kPlanningRadius;
}

// Extra path margin. Open-tier paths sit at least one body radius off
// obstacles. Pinch-tier paths are strictly outside the planning circle;
// margin == 0 is contact with that circle.
inline bool PathClearsOpen(double margin) {
  return std::isfinite(margin) && margin >= kOpenPreferPathMargin;
}
inline bool PathClearsPinch(double margin) {
  return std::isfinite(margin) && margin > kPinchMinPathMargin;
}

}  // namespace navigation2d
