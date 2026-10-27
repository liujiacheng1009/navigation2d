#include "navigation2d/configuration/navigation_yaml.h"

#include "navigation2d/configuration/yaml_schema.h"
namespace navigation2d::configuration_detail {
namespace {
AlgorithmParameterSets ReadParameterSets(const YAML::Node& node, const std::string& path) {
  AlgorithmParameterSets result;
  if (!node) {
    return result;
  }
  if (!node.IsMap()) {
    throw std::runtime_error(path + " must be a mapping");
  }
  for (const auto& entry : node) {
    const auto id = entry.first.as<std::string>();
    if (id.empty() || !entry.second.IsMap() || result.contains(id)) {
      throw std::runtime_error("invalid or duplicate algorithm namespace: " + path + "." + id);
    }
    AlgorithmParameters parameters;
    for (const auto& item : entry.second) {
      const auto key = item.first.as<std::string>();
      if (key.empty() || !item.second.IsScalar() ||
          !parameters.emplace(key, item.second.Scalar()).second) {
        throw std::runtime_error("invalid or duplicate scalar parameter: " + path + "." + id + "." +
                                 key);
      }
    }
    result.emplace(id, std::move(parameters));
  }
  return result;
}
}  // namespace

NavigationConfig ReadNavigationYaml(const std::string& filename) {
  const YAML::Node root = YAML::LoadFile(filename);
  auto legacy = YAML::Clone(root);
  legacy.remove("global_planners");
  legacy.remove("local_controllers");
  ExactKeys(legacy,
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
  c.global_planners = ReadParameterSets(root["global_planners"], "global_planners");
  c.local_controllers = ReadParameterSets(root["local_controllers"], "local_controllers");
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
