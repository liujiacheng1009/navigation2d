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
// Open tier: prefer mid-aisle; thinner open paths fall through to pinch.
inline constexpr double kOpenPreferPathMargin = .22;
// Pinch / doorway: body already fits (planner lethal = kPlanningRadius);
// only refuse tip scrapes with almost no extra margin.
inline constexpr double kPinchMinPathMargin = .15;

}  // namespace navigation2d
