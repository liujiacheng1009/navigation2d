#include "navigation2d/application/navigation_config.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace navigation2d {
namespace {
double Number(const YAML::Node& node, const char* key) {
  if (!node[key]) throw std::runtime_error(std::string("missing configuration key: ") + key);
  const double value = node[key].as<double>();
  if (!std::isfinite(value)) throw std::runtime_error(std::string("non-finite configuration key: ") + key);
  return value;
}
void ExactKeys(const YAML::Node& node, std::initializer_list<const char*> expected,
               const std::string& path) {
  if (!node.IsMap()) throw std::runtime_error(path + " must be a mapping");
  for (const auto& item : node) {
    const std::string key = item.first.as<std::string>();
    bool known = false;
    for (const char* value : expected) known = known || key == value;
    if (!known) throw std::runtime_error("unknown configuration key: " + path + "." + key);
  }
  for (const char* key : expected) if (!node[key])
    throw std::runtime_error("missing configuration key: " + path + "." + key);
}
}

NavigationConfig NavigationConfig::Load(const std::string& filename) {
  const YAML::Node root = YAML::LoadFile(filename);
  ExactKeys(root, {"selection", "map", "costmap", "state_lattice", "path_smoother",
                   "controller", "dwa", "mppi", "mpc",
                   "safety", "scheduler", "recovery"}, "root");
  NavigationConfig c;
  const auto selection = root["selection"];
  ExactKeys(selection, {"planner", "controller"}, "selection");
  c.planner = selection["planner"].as<std::string>();
  c.controller = selection["controller"].as<std::string>();
  const auto map = root["map"];
  ExactKeys(map, {"resolution"}, "map"); c.map_resolution = Number(map, "resolution");
  const auto costmap = root["costmap"];
  ExactKeys(costmap, {"robot_radius", "footprint", "inflation_radius", "inflation_cost_scaling",
                      "obstacle_max_range", "raytrace_max_range", "local_window_width",
                      "local_window_height"}, "costmap");
  c.robot_radius = Number(costmap, "robot_radius");
  if (!costmap["footprint"].IsSequence()) throw std::runtime_error("costmap.footprint must be a sequence");
  for (const auto& point : costmap["footprint"]) {
    if (!point.IsSequence() || point.size() != 2)
      throw std::runtime_error("costmap.footprint points must be [x, y]");
    const double x = point[0].as<double>(), y = point[1].as<double>();
    if (!std::isfinite(x) || !std::isfinite(y))
      throw std::runtime_error("costmap.footprint contains non-finite coordinates");
    c.footprint.emplace_back(x, y);
  }
  c.inflation_radius = Number(costmap, "inflation_radius");
  c.inflation_cost_scaling = Number(costmap, "inflation_cost_scaling");
  c.obstacle_max_range = Number(costmap, "obstacle_max_range");
  c.raytrace_max_range = Number(costmap, "raytrace_max_range");
  c.local_window_width = Number(costmap, "local_window_width");
  c.local_window_height = Number(costmap, "local_window_height");
  const auto lattice = root["state_lattice"];
  ExactKeys(lattice, {"yaw_bins", "primitive_length", "allow_reverse", "reverse_penalty",
                      "rotation_cost", "cost_penalty", "max_expansions", "max_planning_time",
                      "initial_heuristic_weight", "heuristic_weight_decrement"},
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
  const auto smoother = root["path_smoother"];
  ExactKeys(smoother, {"enabled", "max_iterations", "data_weight", "smooth_weight",
                       "obstacle_weight", "min_clearance", "max_deviation", "max_curvature",
                       "max_step", "tolerance"}, "path_smoother");
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
  const auto controller = root["controller"];
  ExactKeys(controller, {"desired_linear_velocity", "lookahead_time",
                          "min_lookahead_distance", "max_lookahead_distance",
                          "rotate_to_heading_min_angle", "min_approach_velocity",
                          "regulated_min_radius", "regulated_min_speed", "max_reverse_velocity",
                          "max_angular_velocity", "max_linear_acceleration",
                          "max_angular_acceleration", "control_period"}, "controller");
  c.desired_linear_velocity = Number(controller, "desired_linear_velocity");
  c.lookahead_time = Number(controller, "lookahead_time");
  c.min_lookahead_distance = Number(controller, "min_lookahead_distance");
  c.max_lookahead_distance = Number(controller, "max_lookahead_distance");
  c.rotate_to_heading_min_angle = Number(controller, "rotate_to_heading_min_angle");
  c.min_approach_velocity = Number(controller, "min_approach_velocity");
  c.regulated_min_radius = Number(controller, "regulated_min_radius");
  c.regulated_min_speed = Number(controller, "regulated_min_speed");
  c.max_reverse_velocity = Number(controller, "max_reverse_velocity");
  c.max_angular_velocity = Number(controller, "max_angular_velocity");
  c.max_linear_acceleration = Number(controller, "max_linear_acceleration");
  c.max_angular_acceleration = Number(controller, "max_angular_acceleration");
  c.control_period = Number(controller, "control_period");
  const auto dwa = root["dwa"];
  ExactKeys(dwa, {"horizon", "linear_samples", "angular_samples", "path_weight",
                  "goal_weight", "obstacle_weight", "velocity_weight"}, "dwa");
  c.dwa_horizon = Number(dwa, "horizon");
  c.dwa_linear_samples = dwa["linear_samples"].as<int>();
  c.dwa_angular_samples = dwa["angular_samples"].as<int>();
  c.dwa_path_weight = Number(dwa, "path_weight");
  c.dwa_goal_weight = Number(dwa, "goal_weight");
  c.dwa_obstacle_weight = Number(dwa, "obstacle_weight");
  c.dwa_velocity_weight = Number(dwa, "velocity_weight");
  const auto mppi = root["mppi"];
  ExactKeys(mppi, {"time_steps", "batch_size", "iterations", "seed", "temperature", "gamma",
                   "vx_std", "wz_std", "constraint_weight", "cost_weight", "goal_weight",
                   "goal_angle_weight", "path_align_weight", "path_follow_weight",
                   "path_angle_weight", "prefer_forward_weight", "smoothness_weight"}, "mppi");
  c.mppi_time_steps = mppi["time_steps"].as<int>();
  c.mppi_batch_size = mppi["batch_size"].as<int>();
  c.mppi_iterations = mppi["iterations"].as<int>();
  c.mppi_seed = mppi["seed"].as<unsigned int>();
  c.mppi_temperature = Number(mppi, "temperature");
  c.mppi_gamma = Number(mppi, "gamma");
  c.mppi_vx_std = Number(mppi, "vx_std");
  c.mppi_wz_std = Number(mppi, "wz_std");
  c.mppi_constraint_weight = Number(mppi, "constraint_weight");
  c.mppi_cost_weight = Number(mppi, "cost_weight");
  c.mppi_goal_weight = Number(mppi, "goal_weight");
  c.mppi_goal_angle_weight = Number(mppi, "goal_angle_weight");
  c.mppi_path_align_weight = Number(mppi, "path_align_weight");
  c.mppi_path_follow_weight = Number(mppi, "path_follow_weight");
  c.mppi_path_angle_weight = Number(mppi, "path_angle_weight");
  c.mppi_prefer_forward_weight = Number(mppi, "prefer_forward_weight");
  c.mppi_smoothness_weight = Number(mppi, "smoothness_weight");
  const auto mpc = root["mpc"];
  ExactKeys(mpc, {"solver", "time_steps", "contour_weight", "heading_weight", "speed_weight",
                  "control_weight", "control_rate_weight", "obstacle_weight", "progress_weight",
                  "max_lateral_acceleration", "dynamic_safety_margin", "dynamic_sigma_scale",
                  "deadline", "dynamic_prediction_timeout"}, "mpc");
  c.mpc_solver = mpc["solver"].as<std::string>();
  c.mpc_time_steps = mpc["time_steps"].as<int>();
  c.mpc_contour_weight = Number(mpc, "contour_weight");
  c.mpc_heading_weight = Number(mpc, "heading_weight");
  c.mpc_speed_weight = Number(mpc, "speed_weight");
  c.mpc_control_weight = Number(mpc, "control_weight");
  c.mpc_control_rate_weight = Number(mpc, "control_rate_weight");
  c.mpc_obstacle_weight = Number(mpc, "obstacle_weight");
  c.mpc_progress_weight = Number(mpc, "progress_weight");
  c.mpc_max_lateral_acceleration = Number(mpc, "max_lateral_acceleration");
  c.mpc_dynamic_safety_margin = Number(mpc, "dynamic_safety_margin");
  c.mpc_dynamic_sigma_scale = Number(mpc, "dynamic_sigma_scale");
  c.mpc_deadline = Number(mpc, "deadline");
  c.dynamic_prediction_timeout = Number(mpc, "dynamic_prediction_timeout");
  const auto safety = root["safety"];
  ExactKeys(safety, {"collision_horizon", "max_navigation_duration", "goal_xy_tolerance", "goal_yaw_tolerance"}, "safety");
  c.collision_horizon = Number(safety, "collision_horizon");
  c.max_navigation_duration = Number(safety, "max_navigation_duration");
  c.goal_xy_tolerance = Number(safety, "goal_xy_tolerance");
  c.goal_yaw_tolerance = Number(safety, "goal_yaw_tolerance");
  const auto scheduler = root["scheduler"];
  ExactKeys(scheduler, {"global_replan_period"}, "scheduler");
  c.global_replan_period = Number(scheduler, "global_replan_period");
  const auto recovery = root["recovery"];
  ExactKeys(recovery, {"progress_radius", "progress_timeout", "duration",
                       "linear_velocity", "angular_velocity", "dynamic_obstacle_radius"}, "recovery");
  c.progress_radius = Number(recovery, "progress_radius");
  c.progress_timeout = Number(recovery, "progress_timeout");
  c.recovery_duration = Number(recovery, "duration");
  c.recovery_linear_velocity = Number(recovery, "linear_velocity");
  c.recovery_angular_velocity = Number(recovery, "angular_velocity");
  c.dynamic_obstacle_radius = Number(recovery, "dynamic_obstacle_radius");
  if ((c.planner != "dijkstra" && c.planner != "astar" && c.planner != "theta_star" &&
       c.planner != "state_lattice") ||
      (c.controller != "rpp" && c.controller != "dwa" && c.controller != "mppi" && c.controller != "mpc"))
    throw std::runtime_error("unknown planner or controller selection");
  if (c.map_resolution <= 0. || c.robot_radius <= 0. || c.inflation_radius < c.robot_radius ||
      c.inflation_cost_scaling <= 0. || c.obstacle_max_range <= 0. ||
      c.raytrace_max_range < c.obstacle_max_range || c.local_window_width <= 0. ||
      c.local_window_height <= 0. ||
      (!c.footprint.empty() && c.footprint.size() < 3) ||
      c.lattice_yaw_bins < 8 || c.lattice_yaw_bins % 2 != 0 ||
      c.lattice_primitive_length < c.map_resolution || c.lattice_reverse_penalty < 1. ||
      c.lattice_rotation_cost <= 0. || c.lattice_cost_penalty < 0. ||
      c.lattice_max_expansions < 1 || c.lattice_max_planning_time <= 0. ||
      c.lattice_initial_heuristic_weight < 1. || c.lattice_heuristic_weight_decrement <= 0. ||
      c.smoother_max_iterations < 1 || c.smoother_data_weight < 0. ||
      c.smoother_smooth_weight < 0. || c.smoother_obstacle_weight < 0. ||
      c.smoother_min_clearance < c.robot_radius || c.smoother_max_deviation < 0. ||
      c.smoother_max_curvature <= 0. || c.smoother_max_step <= 0. ||
      c.smoother_tolerance <= 0. ||
      c.control_period <= 0. || c.global_replan_period < c.control_period ||
      c.mppi_time_steps < 2 || c.mppi_batch_size < 2 || c.mppi_iterations < 1 ||
      c.mppi_temperature <= 0. || c.mppi_vx_std <= 0. || c.mppi_wz_std <= 0. ||
      (c.mpc_solver != "mppi" && c.mpc_solver != "acados") ||
      c.mpc_time_steps < 2 || c.mpc_contour_weight < 0. ||
      c.mpc_heading_weight < 0. || c.mpc_speed_weight < 0. || c.mpc_control_weight < 0. ||
      c.mpc_control_rate_weight < 0. || c.mpc_obstacle_weight < 0. ||
      c.mpc_progress_weight < 0. || c.mpc_max_lateral_acceleration <= 0. ||
      c.mpc_dynamic_safety_margin < 0. || c.mpc_dynamic_sigma_scale < 0. ||
      c.mpc_deadline <= 0. || c.dynamic_prediction_timeout <= 0. ||
      c.dwa_horizon <= 0. || c.dwa_linear_samples < 1 || c.dwa_angular_samples < 1)
    throw std::runtime_error("invalid navigation configuration bounds");
  return c;
}

}  // namespace navigation2d
