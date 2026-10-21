#pragma once

#include <cmath>
#include <utility>
#include <vector>

namespace navigation2d {

// Measured chassis and the metre distances tuned for it. Defaults match the
// values previously written inline. config/navigation2d.yaml replaces them at
// process start; formulas below follow planning_radius and do not have their
// own keys.
struct RobotProfile {
  double planning_radius = 0.22;
  double chassis_radius = 0.207;
  double lidar_forward_offset = 0.18;
  // Radius used only when publishing the inflated costmap debug topic.
  // Planner tiers keep their own inflation in the explorer node.
  double debug_inflation_radius = 0.30;
  double room_pocket_depth = 0.55;
  double same_tip = 0.80;
  double close_standoff = 0.18;
  double preferred_standoff = 0.55;
  double narrow_min_width = 0.32;
  double narrow_max_width = 1.40;
  double visited_viewpoint_radius = 0.45;
  double visited_frontier_radius = 0.60;
  std::vector<std::pair<double, double>> footprint = {
      {-.145, -.147}, {.115, -.147}, {.145, -.132},
      {.145, .132}, {.115, .147}, {-.145, .147}};
};

const RobotProfile& ActiveRobotProfile();
void InstallRobotProfile(RobotProfile profile);

inline double kPlanningRadius() { return ActiveRobotProfile().planning_radius; }
inline double kChassisRadius() { return ActiveRobotProfile().chassis_radius; }
inline double kLidarForwardOffset() {
  return ActiveRobotProfile().lidar_forward_offset;
}
inline double kDebugInflationRadius() {
  return ActiveRobotProfile().debug_inflation_radius;
}
inline double kRoomPocketDepthM() {
  return ActiveRobotProfile().room_pocket_depth;
}
inline double kSameTipM() { return ActiveRobotProfile().same_tip; }
inline double kCloseStandoff() { return ActiveRobotProfile().close_standoff; }
inline double kPreferredStandoff() {
  return ActiveRobotProfile().preferred_standoff;
}
inline double kNarrowMinWidth() { return ActiveRobotProfile().narrow_min_width; }
inline double kNarrowMaxWidth() { return ActiveRobotProfile().narrow_max_width; }
inline double kVisitedViewpointRadius() {
  return ActiveRobotProfile().visited_viewpoint_radius;
}
inline double kVisitedFrontierRadius() {
  return ActiveRobotProfile().visited_frontier_radius;
}

// One planning-circle diameter. Retreat lengths use this as a travel distance.
inline double kSwingClearance() { return 2. * kPlanningRadius(); }

// Cone half-angle whose lateral span at kSwingClearance() equals one body
// radius. With swing = 2 * radius this is atan(1/2), independent of the
// particular chassis.
inline double kBodyConeHalfAngle() {
  return std::atan(kPlanningRadius() / kSwingClearance());
}

inline double kNarrowClearance() { return kPlanningRadius(); }

// Trail-reverse distances, in planning-circle diameters.
inline double kEntryRetreat() { return 2. * kSwingClearance(); }
inline double kOpenRetreat() { return 3. * kSwingClearance(); }
inline double kPocketRetreat() { return 4. * kSwingClearance(); }

inline constexpr double kOpenPreferPathMargin = 0.;
inline constexpr double kPinchMinPathMargin = 0.;
static_assert(kOpenPreferPathMargin == kPinchMinPathMargin);

inline bool HitInsideBody(double range) {
  return range > 0. && range < kPlanningRadius();
}
inline bool HitBlocksSwing(double range) { return HitInsideBody(range); }
inline bool AxisSwingClear(double range) {
  return range >= kPlanningRadius();
}

inline bool PathClearsPinch(double margin) {
  return std::isfinite(margin) && margin > kPinchMinPathMargin;
}
inline bool PathClearsOpen(double margin) { return PathClearsPinch(margin); }

}  // namespace navigation2d
