#include "navigation2d/application/exploration/navigation_profile.h"

#include <stdexcept>

#include "navigation2d/geometry/robot_geometry.h"

namespace navigation2d::exploration {
const AlgorithmSelection& NavigationProfiles::For(NavigationPurpose purpose) const {
  switch (purpose) {
    case NavigationPurpose::kExplore:
      return explore;
    case NavigationPurpose::kEntry:
      return entry;
    case NavigationPurpose::kReturn:
      return returning;
  }
  throw std::invalid_argument("invalid navigation purpose");
}
NavigationConfig BuildNavigationProfile(const NavigationProfileRequest& request,
                                        const AlgorithmSelection& algorithms) {
  navigation2d::NavigationConfig config;
  config.map_resolution = request.resolution;
  // Grid search stays inside the explorer Tick budget.  The SE(2) lattice
  // plus its obstacle heuristic can allocate a yaw-binned graph the size of
  // the whole localization map and never return, so the node produces no
  // result.json.  First-segment cooldown, escape, and A* replay rejection
  // already stop rotate-in-place stalls without that planner.
  // Regulated Pure Pursuit is the Nav2 controller used for robust service
  // robot path following. Its velocity regulation around curvature and
  // obstacles avoids MPPI fallback oscillation on partially observed maps.
  config.controller = algorithms.local;
  // Prefer forward tracking, but allow a short reverse so a U-alcove tip
  // can back out a few tens of centimetres while turning toward the exit.
  config.max_reverse_velocity = .10;
  // CAD: chassis 290×230 mm, wheel envelope half-width 0.147 m, max
  // footprint radius ~0.207 m. Match production navigation2d_node (+margin).
  // Always plan at kPlanningRadius(). The old entry 0.14 m / pinch acceptance
  // <0.15 m let GLOBAL thread furniture V-gaps the body cannot fit.
  const bool entry_drive = request.entry;
  // Entry: A* only. Theta* long chords cut nearby wall tips even when the
  // locked mouth is metres wide (live: gap=2.15 Entry probe rejected
  // path-intersects → abandon → tour loop → failed_goals thrash).
  config.planner = algorithms.global;
  config.robot_radius = navigation2d::kPlanningRadius();
  // Keep the chassis rectangle on a door crawl. Clearing it made the
  // collision monitor take the planning circle as the body width, so a
  // 0.29 m robot was stopped as though it were 0.44 m wide.
  config.footprint.clear();
  for (const auto& point : navigation2d::ActiveRobotProfile().footprint) {
    config.footprint.emplace_back(point.first, point.second);
  }
  // Clearance tiers (strategy, not scene knobs):
  //   entry  — locked mouth crawl (softer inflation only)
  //   pinch  — automatic retry when open-tier finds no route through a
  //            door-width throat (any map, not kitchen-specific)
  //   open   — default explore: prefer mid-aisle over wall feet
  // Lethal radius stays kPlanningRadius() in all tiers; tiers only change
  // soft inflation.
  const bool pinch_plan = request.pinch_retry && !entry_drive && !request.returning;
  // Soft inflation must stay below half a typical doorway (~0.35 m for a
  // 0.70 m mouth). Open 0.42 sealed those doors in cost (UI: tan band fills
  // the gap) and made pinch/open retries report no path.
  if (entry_drive) {
    config.inflation_radius = .26;
    config.inflation_cost_scaling = 8.0;
  } else if (pinch_plan) {
    config.inflation_radius = .28;
    config.inflation_cost_scaling = 7.0;
  } else {
    // Stronger soft inflation than pinch: mid-aisle preference. Doors that
    // open-tier cannot thread still fall through via no-route → pinch.
    config.inflation_radius = .36;
    config.inflation_cost_scaling = 9.0;
  }
  // The global plan already enforces the footprint buffer.  A long local
  // collision projection into sparse unknown cells caused false stops in
  // visibly clear aisles; retain near-field collision checking only.
  config.collision_horizon = .50;
  // Doorway crawl: jamb returns sit at ~body half-width and latched STOP
  // with the open-space travel corridor. Passage mode narrows that band.
  if (entry_drive) {
    // Passage mode: keep monitor on so a true bumper hit still stops, but
    // do not treat door jambs as forward obstacles. stop_distance was 0.02 m
    // (contact) — the mouth-crawl override then shoved 0.16 m/s into the
    // wall after the monitor had already zeroed the command (UI: LOC on wall).
    config.collision_monitor_enabled = true;
    config.collision_monitor_passage_mode = true;
    config.collision_monitor_slowdown_distance = .18;
    config.collision_monitor_approach_horizon = .50;
    config.collision_monitor_stop_distance = .08;
  }
  // Entry crawl must clear the sim/base deadzone. Live: RPP published
  // 0.048 m/s (accel-limited from rest), measured velocity stayed 0, and
  // CommandStalled fired "no measured motion" before the chassis moved.
  config.desired_linear_velocity = entry_drive ? .22 : .28;
  config.regulated_min_speed = entry_drive ? .14 : .10;
  if (entry_drive) {
    config.max_linear_acceleration = 2.4;
    config.regulated_min_radius = .28;
    config.regulated_clearance_min_scale = .90;
    config.regulated_clearance_min_scale_trusted = .95;
    config.rotate_to_heading_min_angle = .85;
    config.progress_timeout = 40.;
  }
  // Soft target in open space: pull paths off speckles/walls when the room
  // is wide. Doorways cannot reach 0.50 m clearance — that smoother pulled
  // entry chords into jambs / PathCuts rejects (live: probe rejected
  // path-intersects while the free throat was clear).
  if (entry_drive) {
    // No soft pull on entry — smoother bows cut wall tips and PathCuts then
    // rejects a perfectly wide mouth (live gap=2.15 thrash).
    config.smoother_enabled = false;
  } else if (pinch_plan) {
    // Enough to thread a certified doorway; not a wall-foot preference.
    config.smoother_min_clearance = .28;
    config.smoother_obstacle_weight = 1.0;
    config.smoother_data_weight = .08;
    config.smoother_max_deviation = .40;
    config.smoother_max_step = .05;
  } else {
    config.smoother_min_clearance = .55;
    config.smoother_obstacle_weight = 1.9;
    config.smoother_data_weight = .04;
    config.smoother_max_deviation = .65;
    config.smoother_max_step = .06;
  }
  // Frontier goals are observation viewpoints, but home is a docking pose.
  // Keep their completion contracts separate: return must converge to the
  // recorded ground-truth start instead of stopping at the edge of the
  // ordinary navigation tolerance disk.
  // Breadcrumb hops are waypoints, not the dock: docking each 0.44 m hop to
  // 4 cm made the return stop-and-creep at every trail sample.
  config.goal_xy_tolerance = request.home_goal ? .04 : .18;
  // LD14 observes 360 degrees, so frontier visits need no stop-and-turn
  // terminal orientation. Preserve precise heading only when docking home.
  // Backtrack edges are positional checkpoints; requiring a docking yaw at
  // every 0.35 m checkpoint creates needless stop-turn-stop motion.
  // Returning means recovering the recorded start point. With a 360-degree
  // lidar there is no mission requirement to reproduce the arbitrary start
  // yaw; enforcing it caused needless terminal spin after position docking.
  config.goal_yaw_tolerance = 3.13;
  return config;
}
}  // namespace navigation2d::exploration
