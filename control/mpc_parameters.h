#pragma once
#include <string>

#include "navigation2d/control/dynamic_safety_parameters.h"

namespace navigation2d {
// Cascade policy and safety input; child optimizers own their own parameters.
struct MpcParameters {
  std::string mpc_solver;
  int guidance_max_candidates;
  double guidance_candidate_timeout;
  DynamicSafetyParameters safety;
};
MpcParameters MpcParametersFrom(const NavigationConfig& config);
}  // namespace navigation2d
