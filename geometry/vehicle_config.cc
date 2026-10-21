#include "navigation2d/geometry/vehicle_config.h"

#include "navigation2d/geometry/robot_geometry.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <yaml-cpp/yaml.h>

namespace navigation2d {
namespace {

RobotProfile& MutableRobotProfile() {
  static RobotProfile profile;
  return profile;
}

double Number(const YAML::Node& node, const char* key) {
  if (!node[key])
    throw std::runtime_error(std::string("missing configuration key: ") + key);
  const double value = node[key].as<double>();
  if (!std::isfinite(value))
    throw std::runtime_error(std::string("non-finite configuration key: ") + key);
  return value;
}

void ExactKeys(const YAML::Node& node, std::initializer_list<const char*> expected,
               const std::string& path) {
  if (!node.IsMap()) throw std::runtime_error(path + " must be a mapping");
  for (const auto& item : node) {
    const std::string key = item.first.as<std::string>();
    bool known = false;
    for (const char* value : expected) known = known || key == value;
    if (!known)
      throw std::runtime_error("unknown configuration key: " + path + "." + key);
  }
  for (const char* key : expected)
    if (!node[key])
      throw std::runtime_error("missing configuration key: " + path + "." + key);
}

void Positive(double value, const char* key) {
  if (!(value > 0.))
    throw std::runtime_error(std::string(key) + " must be positive");
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
  if (!(profile.chassis_radius < profile.planning_radius))
    throw std::runtime_error(
        "chassis_radius_m must be inside planning_radius_m");
  if (!(profile.narrow_min_width < profile.narrow_max_width))
    throw std::runtime_error(
        "narrow_min_width_m must be below narrow_max_width_m");
  if (profile.footprint.size() < 3)
    throw std::runtime_error("footprint_m needs at least 3 points");
  MutableRobotProfile() = std::move(profile);
}

void ApplyVehicleConfig(const std::string& path, FrontierExplorerConfig& config) {
  const YAML::Node root = YAML::LoadFile(path);
  // Same file as the planner (config/navigation2d.yaml). Sibling sections
  // belong to NavigationConfig::Load; only robot and exploration are read here.
  if (!root.IsMap()) throw std::runtime_error("root must be a mapping");
  if (!root["robot"]) throw std::runtime_error("missing configuration key: root.robot");
  if (!root["exploration"])
    throw std::runtime_error("missing configuration key: root.exploration");
  const auto robot = root["robot"];
  ExactKeys(robot,
            {"planning_radius_m", "chassis_radius_m", "lidar_forward_offset_m",
             "debug_inflation_radius_m", "footprint_m"},
            "robot");
  const auto exploration = root["exploration"];
  ExactKeys(exploration,
            {"footprint_clearance_m", "minimum_standoff_m", "maximum_standoff_m",
             "blacklist_radius_m", "frontier_stability_radius_m",
             "close_standoff_m", "preferred_standoff_m", "narrow_min_width_m",
             "narrow_max_width_m", "visited_viewpoint_radius_m",
             "visited_frontier_radius_m", "room_pocket_depth_m", "same_tip_m"},
            "exploration");

  RobotProfile profile;
  profile.planning_radius = Number(robot, "planning_radius_m");
  profile.chassis_radius = Number(robot, "chassis_radius_m");
  profile.lidar_forward_offset = Number(robot, "lidar_forward_offset_m");
  profile.debug_inflation_radius = Number(robot, "debug_inflation_radius_m");
  if (!robot["footprint_m"].IsSequence())
    throw std::runtime_error("robot.footprint_m must be a sequence");
  profile.footprint.clear();
  for (const auto& point : robot["footprint_m"]) {
    if (!point.IsSequence() || point.size() != 2)
      throw std::runtime_error("robot.footprint_m points must be [x, y]");
    const double x = point[0].as<double>();
    const double y = point[1].as<double>();
    if (!std::isfinite(x) || !std::isfinite(y))
      throw std::runtime_error("robot.footprint_m contains a non-finite point");
    profile.footprint.emplace_back(x, y);
  }
  profile.room_pocket_depth = Number(exploration, "room_pocket_depth_m");
  profile.same_tip = Number(exploration, "same_tip_m");
  profile.close_standoff = Number(exploration, "close_standoff_m");
  profile.preferred_standoff = Number(exploration, "preferred_standoff_m");
  profile.narrow_min_width = Number(exploration, "narrow_min_width_m");
  profile.narrow_max_width = Number(exploration, "narrow_max_width_m");
  profile.visited_viewpoint_radius =
      Number(exploration, "visited_viewpoint_radius_m");
  profile.visited_frontier_radius =
      Number(exploration, "visited_frontier_radius_m");
  InstallRobotProfile(profile);

  config.footprint_clearance = Number(exploration, "footprint_clearance_m");
  config.minimum_standoff = Number(exploration, "minimum_standoff_m");
  config.maximum_standoff = Number(exploration, "maximum_standoff_m");
  config.blacklist_radius = Number(exploration, "blacklist_radius_m");
  config.frontier_stability_radius =
      Number(exploration, "frontier_stability_radius_m");
  Positive(config.footprint_clearance, "footprint_clearance_m");
  Positive(config.minimum_standoff, "minimum_standoff_m");
  Positive(config.maximum_standoff, "maximum_standoff_m");
  Positive(config.blacklist_radius, "blacklist_radius_m");
  Positive(config.frontier_stability_radius, "frontier_stability_radius_m");
  if (!(config.minimum_standoff < config.maximum_standoff))
    throw std::runtime_error(
        "minimum_standoff_m must be below maximum_standoff_m");
}

}  // namespace navigation2d
