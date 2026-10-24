#include "navigation2d/control/pursuit_parameters.h"

#include "navigation2d/configuration/navigation_config.h"
namespace navigation2d::pursuit_internal {
Parameters ParametersFrom(const NavigationConfig& config) {
  Parameters result;
  result.tracking.collision = {config.robot_radius, config.collision_horizon,
                               config.control_period};
  result.tracking.regulation.desired_linear_velocity = config.desired_linear_velocity;
  result.tracking.regulation.max_angular_velocity = config.max_angular_velocity;
  result.tracking.regulation.min_approach_velocity = config.min_approach_velocity;
  result.tracking.regulation.regulated_clearance_min_scale = config.regulated_clearance_min_scale;
  result.tracking.regulation.regulated_clearance_min_scale_trusted =
      config.regulated_clearance_min_scale_trusted;
  result.tracking.regulation.regulated_forward_trust_half_width =
      config.regulated_forward_trust_half_width;
  result.tracking.regulation.regulated_forward_trust_length = config.regulated_forward_trust_length;
  result.tracking.regulation.regulated_min_radius = config.regulated_min_radius;
  result.tracking.regulation.regulated_min_speed = config.regulated_min_speed;
  result.tracking.regulation.regulated_soft_clearance = config.regulated_soft_clearance;
  result.tracking.regulation.regulated_soft_clearance_trusted =
      config.regulated_soft_clearance_trusted;
  result.tracking.regulation.robot_radius = config.robot_radius;
  result.tracking.regulation.rotate_to_heading_min_angle = config.rotate_to_heading_min_angle;
  result.tracking.max_linear_acceleration = config.max_linear_acceleration;
  result.tracking.max_angular_acceleration = config.max_angular_acceleration;
  result.tracking.lookahead_time = config.lookahead_time;
  result.tracking.min_lookahead_distance = config.min_lookahead_distance;
  result.tracking.max_lookahead_distance = config.max_lookahead_distance;
  result.alignment.collision = result.tracking.collision;
  result.alignment.max_linear_acceleration = config.max_linear_acceleration;
  result.alignment.max_angular_acceleration = config.max_angular_acceleration;
  result.alignment.max_angular_velocity = config.max_angular_velocity;
  result.alignment.regulated_min_speed = config.regulated_min_speed;
  result.alignment.max_lookahead_distance = config.max_lookahead_distance;
  return result;
}
}  // namespace navigation2d::pursuit_internal
