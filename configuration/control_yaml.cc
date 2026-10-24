#include "navigation2d/configuration/navigation_yaml.h"
#include "navigation2d/configuration/yaml_schema.h"
namespace navigation2d::configuration_detail {
namespace {
void ReadRpp(const YAML::Node& root, NavigationConfig& c) {
  const auto controller = root["controller"];
  ExactKeys(controller,
            {"desired_linear_velocity", "lookahead_time", "min_lookahead_distance",
             "max_lookahead_distance", "rotate_to_heading_min_angle", "min_approach_velocity",
             "regulated_min_radius", "regulated_min_speed", "max_reverse_velocity",
             "max_angular_velocity", "max_linear_acceleration", "max_angular_acceleration",
             "control_period"},
            "controller");
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
}
void ReadDwa(const YAML::Node& root, NavigationConfig& c) {
  const auto dwa = root["dwa"];
  ExactKeys(dwa,
            {"horizon", "linear_samples", "angular_samples", "path_weight", "goal_weight",
             "obstacle_weight", "velocity_weight"},
            "dwa");
  c.dwa_horizon = Number(dwa, "horizon");
  c.dwa_linear_samples = dwa["linear_samples"].as<int>();
  c.dwa_angular_samples = dwa["angular_samples"].as<int>();
  c.dwa_path_weight = Number(dwa, "path_weight");
  c.dwa_goal_weight = Number(dwa, "goal_weight");
  c.dwa_obstacle_weight = Number(dwa, "obstacle_weight");
  c.dwa_velocity_weight = Number(dwa, "velocity_weight");
}
void ReadMppi(const YAML::Node& root, NavigationConfig& c) {
  const auto mppi = root["mppi"];
  ExactKeys(
      mppi,
      {"time_steps", "batch_size", "iterations", "seed", "temperature", "gamma", "vx_std", "wz_std",
       "constraint_weight", "cost_weight", "goal_weight", "goal_angle_weight", "path_align_weight",
       "path_follow_weight", "path_angle_weight", "prefer_forward_weight", "smoothness_weight"},
      "mppi");
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
}
void ReadMpc(const YAML::Node& root, NavigationConfig& c) {
  const auto mpc = root["mpc"];
  ExactKeys(mpc,
            {"solver", "time_steps", "contour_weight", "heading_weight", "speed_weight",
             "control_weight", "control_rate_weight", "obstacle_weight", "progress_weight",
             "max_lateral_acceleration", "dynamic_safety_margin", "dynamic_sigma_scale", "deadline",
             "dynamic_prediction_timeout", "guidance_max_candidates", "guidance_candidate_timeout"},
            "mpc");
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
  c.guidance_max_candidates = mpc["guidance_max_candidates"].as<int>();
  c.guidance_candidate_timeout = Number(mpc, "guidance_candidate_timeout");
}
}  // namespace
void ReadControlYaml(const YAML::Node& root, NavigationConfig& c) {
  ReadRpp(root, c);
  ReadDwa(root, c);
  ReadMppi(root, c);
  ReadMpc(root, c);
}
}  // namespace navigation2d::configuration_detail
