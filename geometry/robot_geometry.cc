#include "navigation2d/geometry/robot_geometry.h"

#include <stdexcept>
#include <string>
#include <utility>

namespace navigation2d {
namespace {
RobotProfile& MutableRobotProfile() {
  static RobotProfile profile;
  return profile;
}

void Positive(double value, const char* key) {
  if (!(value > 0.)) {
    throw std::runtime_error(std::string(key) + " must be positive");
  }
}

}  // namespace

const RobotProfile& ActiveRobotProfile() { return MutableRobotProfile(); }

void InstallRobotProfile(RobotProfile profile) {
  Positive(profile.planning_radius, "planning_radius_m");
  Positive(profile.chassis_radius, "chassis_radius_m");
  Positive(profile.lidar_forward_offset, "lidar_forward_offset_m");
  Positive(profile.debug_inflation_radius, "debug_inflation_radius_m");
  Positive(profile.room_pocket_depth, "room_pocket_depth_m");
  Positive(profile.same_tip, "same_tip_m");
  Positive(profile.close_standoff, "close_standoff_m");
  Positive(profile.preferred_standoff, "preferred_standoff_m");
  Positive(profile.narrow_min_width, "narrow_min_width_m");
  Positive(profile.narrow_max_width, "narrow_max_width_m");
  Positive(profile.visited_viewpoint_radius, "visited_viewpoint_radius_m");
  Positive(profile.visited_frontier_radius, "visited_frontier_radius_m");
  if (!(profile.chassis_radius < profile.planning_radius)) {
    throw std::runtime_error("chassis_radius_m must be inside planning_radius_m");
  }
  if (!(profile.narrow_min_width < profile.narrow_max_width)) {
    throw std::runtime_error("narrow_min_width_m must be below narrow_max_width_m");
  }
  if (profile.footprint.size() < 3) {
    throw std::runtime_error("footprint_m needs at least 3 points");
  }
  MutableRobotProfile() = std::move(profile);
}

}  // namespace navigation2d
