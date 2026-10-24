#include "navigation2d/configuration/navigation_yaml.h"

#include "navigation2d/configuration/yaml_schema.h"
namespace navigation2d::configuration_detail {
NavigationConfig ReadNavigationYaml(const std::string& filename) {
  const YAML::Node root = YAML::LoadFile(filename);
  ExactKeys(root,
            {"selection", "map", "costmap", "state_lattice", "path_smoother", "controller", "dwa",
             "mppi", "mpc", "safety", "scheduler", "recovery", "robot", "exploration"},
            "root");
  ExactKeys(root["robot"],
            {"planning_radius_m", "chassis_radius_m", "lidar_forward_offset_m",
             "debug_inflation_radius_m", "footprint_m"},
            "robot");
  if (!root["robot"]["footprint_m"].IsSequence()) {
    throw std::runtime_error("robot.footprint_m must be a sequence");
  }
  ExactKeys(
      root["exploration"],
      {"footprint_clearance_m", "minimum_standoff_m", "maximum_standoff_m", "blacklist_radius_m",
       "frontier_stability_radius_m", "close_standoff_m", "preferred_standoff_m",
       "narrow_min_width_m", "narrow_max_width_m", "visited_viewpoint_radius_m",
       "visited_frontier_radius_m", "room_pocket_depth_m", "same_tip_m"},
      "exploration");
  NavigationConfig c;
  const auto selection = root["selection"];
  ExactKeys(selection, {"planner", "controller"}, "selection");
  c.planner = selection["planner"].as<std::string>();
  c.controller = selection["controller"].as<std::string>();
  ReadPlanningYaml(root, c);
  ReadControlYaml(root, c);
  ReadSafetyYaml(root, c);
  return c;
}
}  // namespace navigation2d::configuration_detail
