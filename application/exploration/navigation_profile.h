#pragma once
#include <string>

#include "navigation2d/configuration/navigation_config.h"

namespace navigation2d::exploration {
enum class NavigationPurpose { kExplore, kEntry, kReturn };
struct AlgorithmSelection {
  std::string global = "theta_star";
  std::string local = "rpp";
  AlgorithmParameterSets global_parameters{};
  AlgorithmParameterSets local_parameters{};
};
struct NavigationProfiles {
  AlgorithmSelection explore;
  AlgorithmSelection entry{"astar", "rpp"};
  AlgorithmSelection returning;
  const AlgorithmSelection& For(NavigationPurpose purpose) const;
};
struct NavigationProfileRequest {
  double resolution = .03;
  bool entry = false;
  bool pinch_retry = false;
  bool returning = false;
  bool home_goal = false;
};
// Strategy configuration only. Does not mutate maps, tasks or algorithm state.
NavigationConfig BuildNavigationProfile(const NavigationProfileRequest& request,
                                        const AlgorithmSelection& algorithms);
}  // namespace navigation2d::exploration
