#pragma once

#include <cmath>

namespace navigation2d {

// Chassis outline. Lateral half-width is what limits a straight doorway;
// the corner radius is what limits a turn or an unknown heading.
// Footprint (metres): rear/front ±kBodyHalfLength, sides ±kBodyHalfWidth,
// nose chamfer at (kBodyNoseChamferX, ±kBodyNoseChamferY).
inline constexpr double kBodyHalfLength = .145;
inline constexpr double kBodyHalfWidth = .147;
inline constexpr double kBodyNoseChamferX = .115;
inline constexpr double kBodyNoseChamferY = .132;
constexpr double DetailSqrt(double x) {
  double guess = x;
  for (int i = 0; i < 12; ++i) guess = 0.5 * (guess + x / guess);
  return guess;
}

// Corner of that rectangle. Open-space planning uses this circle so a
// diagonal approach cannot clip. Door passage uses kBodyHalfWidth.
inline constexpr double kCornerRadius = DetailSqrt(
    kBodyHalfLength * kBodyHalfLength + kBodyHalfWidth * kBodyHalfWidth);
inline constexpr double kPlanningRadius = kCornerRadius;

// One corner-circle diameter of free space. A lidar hit closer than this
// means the chassis cannot yaw clear of that obstacle. At or beyond it,
// that axis is open. Not a room or doorway width.
inline constexpr double kSwingClearance = 2. * kPlanningRadius;

// Cone half-angle whose lateral span at kSwingClearance equals one body
// radius: atan(kPlanningRadius / kSwingClearance) = atan(1/2).
// Every forward/side stall sample uses this aperture.
inline constexpr double kBodyConeHalfAngle =
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

// Trail-reverse distances, in corner-circle diameters.
inline constexpr double kEntryRetreat = 2. * kSwingClearance;
inline constexpr double kOpenRetreat = 3. * kSwingClearance;
inline constexpr double kPocketRetreat = 4. * kSwingClearance;

// Lidar range is distance to the hit. 0 means that sector returned nothing.
inline bool HitInsideBody(double range) {
  return range > 0. && range < kPlanningRadius;
}
inline bool HitBlocksSwing(double range) {
  return range > 0. && range < kSwingClearance;
}
inline bool AxisSwingClear(double range) {
  return range >= kSwingClearance;
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
