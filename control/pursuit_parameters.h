#pragma once
namespace navigation2d {
struct NavigationConfig;
namespace pursuit_internal {
struct CollisionParameters {
  double robot_radius = 0., collision_horizon = 0., control_period = 0.;
};
struct RegulationParameters {
  double desired_linear_velocity = 0.;
  double max_angular_velocity = 0.;
  double min_approach_velocity = 0.;
  double regulated_clearance_min_scale = 0.;
  double regulated_clearance_min_scale_trusted = 0.;
  double regulated_forward_trust_half_width = 0.;
  double regulated_forward_trust_length = 0.;
  double regulated_min_radius = 0.;
  double regulated_min_speed = 0.;
  double regulated_soft_clearance = 0.;
  double regulated_soft_clearance_trusted = 0.;
  double robot_radius = 0.;
  double rotate_to_heading_min_angle = 0.;
};
struct TrackingParameters {
  CollisionParameters collision;
  RegulationParameters regulation;
  double max_linear_acceleration = 0., max_angular_acceleration = 0.;
  double lookahead_time = 0., min_lookahead_distance = 0., max_lookahead_distance = 0.;
};
struct AlignmentParameters {
  CollisionParameters collision;
  double max_linear_acceleration = 0., max_angular_acceleration = 0.;
  double max_angular_velocity = 0., regulated_min_speed = 0., max_lookahead_distance = 0.;
};
struct Parameters {
  TrackingParameters tracking;
  AlignmentParameters alignment;
};
// Value conversion only. The caller keeps the existing NavigationConfig
// validation boundary; the algorithm does not parse YAML or own unused fields.
Parameters ParametersFrom(const NavigationConfig&);
}  // namespace pursuit_internal
}  // namespace navigation2d
