#include "navigation2d/configuration/builtin_parameters.h"

#include <span>
#include <string_view>
#include <variant>

#include "navigation2d/application/builtin_algorithm_catalog.h"
#include "navigation2d/configuration/navigation_validation.h"
namespace navigation2d {
namespace {
using Member = std::variant<double NavigationConfig::*, int NavigationConfig::*,
                            unsigned int NavigationConfig::*, bool NavigationConfig::*,
                            std::string NavigationConfig::*>;
struct Field {
  std::string_view key;
  Member member;
};
const Field kLattice[] = {
    {"yaw_bins", &NavigationConfig::lattice_yaw_bins},
    {"primitive_length", &NavigationConfig::lattice_primitive_length},
    {"allow_reverse", &NavigationConfig::lattice_allow_reverse},
    {"reverse_penalty", &NavigationConfig::lattice_reverse_penalty},
    {"rotation_cost", &NavigationConfig::lattice_rotation_cost},
    {"cost_penalty", &NavigationConfig::lattice_cost_penalty},
    {"max_expansions", &NavigationConfig::lattice_max_expansions},
    {"max_planning_time", &NavigationConfig::lattice_max_planning_time},
    {"initial_heuristic_weight", &NavigationConfig::lattice_initial_heuristic_weight},
    {"heuristic_weight_decrement", &NavigationConfig::lattice_heuristic_weight_decrement},
};
const Field kRpp[] = {
    {"desired_linear_velocity", &NavigationConfig::desired_linear_velocity},
    {"lookahead_time", &NavigationConfig::lookahead_time},
    {"min_lookahead_distance", &NavigationConfig::min_lookahead_distance},
    {"max_lookahead_distance", &NavigationConfig::max_lookahead_distance},
    {"rotate_to_heading_min_angle", &NavigationConfig::rotate_to_heading_min_angle},
    {"min_approach_velocity", &NavigationConfig::min_approach_velocity},
    {"regulated_min_radius", &NavigationConfig::regulated_min_radius},
    {"regulated_min_speed", &NavigationConfig::regulated_min_speed},
};
const Field kDwa[] = {
    {"horizon", &NavigationConfig::dwa_horizon},
    {"linear_samples", &NavigationConfig::dwa_linear_samples},
    {"angular_samples", &NavigationConfig::dwa_angular_samples},
    {"path_weight", &NavigationConfig::dwa_path_weight},
    {"goal_weight", &NavigationConfig::dwa_goal_weight},
    {"obstacle_weight", &NavigationConfig::dwa_obstacle_weight},
    {"velocity_weight", &NavigationConfig::dwa_velocity_weight},
};
const Field kMppi[] = {
    {"time_steps", &NavigationConfig::mppi_time_steps},
    {"batch_size", &NavigationConfig::mppi_batch_size},
    {"iterations", &NavigationConfig::mppi_iterations},
    {"seed", &NavigationConfig::mppi_seed},
    {"temperature", &NavigationConfig::mppi_temperature},
    {"gamma", &NavigationConfig::mppi_gamma},
    {"vx_std", &NavigationConfig::mppi_vx_std},
    {"wz_std", &NavigationConfig::mppi_wz_std},
    {"constraint_weight", &NavigationConfig::mppi_constraint_weight},
    {"cost_weight", &NavigationConfig::mppi_cost_weight},
    {"goal_weight", &NavigationConfig::mppi_goal_weight},
    {"goal_angle_weight", &NavigationConfig::mppi_goal_angle_weight},
    {"path_align_weight", &NavigationConfig::mppi_path_align_weight},
    {"path_follow_weight", &NavigationConfig::mppi_path_follow_weight},
    {"path_angle_weight", &NavigationConfig::mppi_path_angle_weight},
    {"prefer_forward_weight", &NavigationConfig::mppi_prefer_forward_weight},
    {"smoothness_weight", &NavigationConfig::mppi_smoothness_weight},
};
const Field kMpc[] = {
    {"solver", &NavigationConfig::mpc_solver},
    {"time_steps", &NavigationConfig::mpc_time_steps},
    {"contour_weight", &NavigationConfig::mpc_contour_weight},
    {"heading_weight", &NavigationConfig::mpc_heading_weight},
    {"speed_weight", &NavigationConfig::mpc_speed_weight},
    {"control_weight", &NavigationConfig::mpc_control_weight},
    {"control_rate_weight", &NavigationConfig::mpc_control_rate_weight},
    {"obstacle_weight", &NavigationConfig::mpc_obstacle_weight},
    {"progress_weight", &NavigationConfig::mpc_progress_weight},
    {"max_lateral_acceleration", &NavigationConfig::mpc_max_lateral_acceleration},
    {"dynamic_safety_margin", &NavigationConfig::mpc_dynamic_safety_margin},
    {"dynamic_sigma_scale", &NavigationConfig::mpc_dynamic_sigma_scale},
    {"deadline", &NavigationConfig::mpc_deadline},
    {"dynamic_prediction_timeout", &NavigationConfig::dynamic_prediction_timeout},
    {"guidance_max_candidates", &NavigationConfig::guidance_max_candidates},
    {"guidance_candidate_timeout", &NavigationConfig::guidance_candidate_timeout},
};
std::span<const Field> Schema(const std::string& id, bool planner) {
  if (planner) {
    if (id == "state_lattice") {
      return kLattice;
    }
    if (builtin_algorithms::HasPlanner(id)) {
      return {};
    }
  } else {
    if (id == "rpp") {
      return kRpp;
    }
    if (id == "dwa") {
      return kDwa;
    }
    if (id == "mppi") {
      return kMppi;
    }
    if (id == "mpc") {
      return kMpc;
    }
  }
  throw std::invalid_argument("unknown builtin algorithm namespace: " + id);
}
}  // namespace
NavigationConfig ResolveBuiltinParameters(const NavigationConfig& common, const std::string& id,
                                          bool planner) {
  const auto schema = Schema(id, planner);
  NavigationConfig result = common;
  const auto& parameters =
      ParametersFor(planner ? common.global_planners : common.local_controllers, id);
  for (const auto& [key, value] : parameters) {
    const Field* field = nullptr;
    for (const auto& candidate : schema) {
      if (candidate.key == key) {
        field = &candidate;
        break;
      }
    }
    const auto path =
        std::string(planner ? "global_planners." : "local_controllers.") + id + "." + key;
    if (!field) {
      throw std::invalid_argument("unknown algorithm parameter: " + path);
    }
    std::visit(
        [&](auto member) {
          using Value = std::remove_cvref_t<decltype(result.*member)>;
          const auto parsed = ReadAlgorithmParameter<Value>(value, path);
          if constexpr (std::is_signed_v<Value>) {
            if (parsed < 0) {
              throw std::invalid_argument("negative algorithm parameter: " + path);
            }
          }
          result.*member = parsed;
        },
        field->member);
  }
  if (!parameters.empty()) {
    configuration_detail::ValidateNavigationBounds(result);
    if (result.desired_linear_velocity <= 0. || result.max_angular_velocity <= 0. ||
        result.max_linear_acceleration <= 0. || result.max_angular_acceleration <= 0. ||
        result.max_reverse_velocity < 0. || result.min_lookahead_distance <= 0. ||
        result.max_lookahead_distance < result.min_lookahead_distance ||
        result.lookahead_time <= 0.) {
      throw std::invalid_argument("invalid algorithm motion bounds: " + id);
    }
  }
  return result;
}
void ValidateBuiltinParameterSets(const NavigationConfig& config) {
  for (const auto& [id, parameters] : config.global_planners) {
    (void)parameters;
    static_cast<void>(ResolveBuiltinParameters(config, id, true));
  }
  for (const auto& [id, parameters] : config.local_controllers) {
    (void)parameters;
    static_cast<void>(ResolveBuiltinParameters(config, id, false));
  }
}
}  // namespace navigation2d
