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
// Open tier: prefer mid-aisle (hard accept). Thin open paths cool signature
// and retry open; pinch only for no-route or PassableMouth scrape.
inline constexpr double kOpenPreferPathMargin = .22;
// Pinch / doorway: body already fits (planner lethal = kPlanningRadius).
// A 0.70 m door has half-width 0.35 → margin ≈ 0.13 with kPlanningRadius;
// the old 0.15 floor rejected those doors as "wall-scrape". Keep this low
// enough for ~0.60 m mouths (margin ≈ 0.08) but still refuse tip scrapes.
inline constexpr double kPinchMinPathMargin = .08;

}  // namespace navigation2d
