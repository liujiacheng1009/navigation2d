#pragma once

#include <Eigen/Core>
#include <vector>

#include "navigation2d/planning/smoother_parameters.h"

namespace navigation2d {
// Own only lattice geometry, search budgets/costs and its postprocessing input.
struct StateLatticeParameters {
  double robot_radius;
  std::vector<Eigen::Vector2d> footprint;
  int lattice_yaw_bins;
  double lattice_primitive_length;
  bool lattice_allow_reverse;
  double lattice_reverse_penalty;
  double lattice_rotation_cost;
  double lattice_cost_penalty;
  int lattice_max_expansions;
  double lattice_max_planning_time;
  double lattice_initial_heuristic_weight;
  double lattice_heuristic_weight_decrement;
  planning_internal::SmootherParameters smoothing;
};
StateLatticeParameters StateLatticeParametersFrom(const NavigationConfig& config);
}  // namespace navigation2d
