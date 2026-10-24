#include "navigation2d/configuration/navigation_yaml.h"
#include "navigation2d/configuration/yaml_schema.h"
namespace navigation2d::configuration_detail {
namespace {
void ReadMap(const YAML::Node& root, NavigationConfig& c) {
  const auto map = root["map"];
  ExactKeys(map, {"resolution"}, "map");
  c.map_resolution = Number(map, "resolution");
}
void ReadCostmap(const YAML::Node& root, NavigationConfig& c) {
  const auto costmap = root["costmap"];
  ExactKeys(
      costmap,
      {"robot_radius", "footprint", "inflation_radius", "inflation_cost_scaling",
       "obstacle_max_range", "raytrace_max_range", "local_window_width", "local_window_height"},
      "costmap");
  c.robot_radius = Number(costmap, "robot_radius");
  if (!costmap["footprint"].IsSequence()) {
    throw std::runtime_error("costmap.footprint must be a sequence");
  }
  for (const auto& point : costmap["footprint"]) {
    if (!point.IsSequence() || point.size() != 2) {
      throw std::runtime_error("costmap.footprint points must be [x, y]");
    }
    const double x = point[0].as<double>(), y = point[1].as<double>();
    if (!std::isfinite(x) || !std::isfinite(y)) {
      throw std::runtime_error("costmap.footprint contains non-finite coordinates");
    }
    c.footprint.emplace_back(x, y);
  }
  c.inflation_radius = Number(costmap, "inflation_radius");
  c.inflation_cost_scaling = Number(costmap, "inflation_cost_scaling");
  c.obstacle_max_range = Number(costmap, "obstacle_max_range");
  c.raytrace_max_range = Number(costmap, "raytrace_max_range");
  c.local_window_width = Number(costmap, "local_window_width");
  c.local_window_height = Number(costmap, "local_window_height");
}
void ReadLattice(const YAML::Node& root, NavigationConfig& c) {
  const auto lattice = root["state_lattice"];
  ExactKeys(lattice,
            {"yaw_bins", "primitive_length", "allow_reverse", "reverse_penalty", "rotation_cost",
             "cost_penalty", "max_expansions", "max_planning_time", "initial_heuristic_weight",
             "heuristic_weight_decrement"},
            "state_lattice");
  c.lattice_yaw_bins = lattice["yaw_bins"].as<int>();
  c.lattice_primitive_length = Number(lattice, "primitive_length");
  c.lattice_allow_reverse = lattice["allow_reverse"].as<bool>();
  c.lattice_reverse_penalty = Number(lattice, "reverse_penalty");
  c.lattice_rotation_cost = Number(lattice, "rotation_cost");
  c.lattice_cost_penalty = Number(lattice, "cost_penalty");
  c.lattice_max_expansions = lattice["max_expansions"].as<int>();
  c.lattice_max_planning_time = Number(lattice, "max_planning_time");
  c.lattice_initial_heuristic_weight = Number(lattice, "initial_heuristic_weight");
  c.lattice_heuristic_weight_decrement = Number(lattice, "heuristic_weight_decrement");
}
void ReadSmoother(const YAML::Node& root, NavigationConfig& c) {
  const auto smoother = root["path_smoother"];
  ExactKeys(smoother,
            {"enabled", "max_iterations", "data_weight", "smooth_weight", "obstacle_weight",
             "min_clearance", "max_deviation", "max_curvature", "max_step", "tolerance"},
            "path_smoother");
  c.smoother_enabled = smoother["enabled"].as<bool>();
  c.smoother_max_iterations = smoother["max_iterations"].as<int>();
  c.smoother_data_weight = Number(smoother, "data_weight");
  c.smoother_smooth_weight = Number(smoother, "smooth_weight");
  c.smoother_obstacle_weight = Number(smoother, "obstacle_weight");
  c.smoother_min_clearance = Number(smoother, "min_clearance");
  c.smoother_max_deviation = Number(smoother, "max_deviation");
  c.smoother_max_curvature = Number(smoother, "max_curvature");
  c.smoother_max_step = Number(smoother, "max_step");
  c.smoother_tolerance = Number(smoother, "tolerance");
}
}  // namespace
void ReadPlanningYaml(const YAML::Node& root, NavigationConfig& c) {
  ReadMap(root, c);
  ReadCostmap(root, c);
  ReadLattice(root, c);
  ReadSmoother(root, c);
}
}  // namespace navigation2d::configuration_detail
