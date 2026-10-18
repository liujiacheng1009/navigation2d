#pragma once

namespace navigation2d {

// Inscribed planning radius for the chassis (CAD max footprint ~0.207 m +
// margin). Explorer nomination, PathCuts, and autonomous explore Navigation
// configs share this value — do not scatter bare 0.22 literals.
inline constexpr double kPlanningRadius = .22;

// NavigationSystem::AddPathMetrics reports
//   path_min_clearance_m = dist_to_obstacle - robot_radius
// i.e. EXTRA free margin beyond the body — not the raw obstacle distance.
// Comparing that field to kPlanningRadius would wrongly demand ~0.44 m to
// walls (corridor width ~0.88 m) and reject ordinary indoor aisles.
//
// Bottleneck-aware acceptance (AnalyzePathClearanceProfile):
//   free_width >= kOpenFreeWidthMin → require kOpenPreferPathMargin
//   free_width <  kOpenFreeWidthMin → allow down to kPinchMinPathMargin
// Whole-path pinch downgrade is only for no-route, not open wall scrapes.
inline constexpr double kOpenPreferPathMargin = .22;
// Pinch / doorway: body already fits (planner lethal = kPlanningRadius).
// A 0.70 m door has half-width 0.35 → margin ≈ 0.13 with kPlanningRadius;
// the old 0.15 floor rejected those doors as "wall-scrape". Keep this low
// enough for ~0.60 m mouths (margin ≈ 0.08) but still refuse tip scrapes.
inline constexpr double kPinchMinPathMargin = .08;
// Perpendicular free span that counts as "open aisle" (not a door throat).
// Wall-foot chords in rooms this wide must keep open margin.
inline constexpr double kOpenFreeWidthMin = 1.00;
// Typical door/alcove free span upper band (documentation + tests). Samples
// thinner than kOpenFreeWidthMin are bottleneck-eligible regardless.
inline constexpr double kBottleneckFreeWidthMax = .95;

}  // namespace navigation2d
