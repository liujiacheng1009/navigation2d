#pragma once
#include <Eigen/Core>
#include <vector>

namespace navigation2d {
struct NavigationConfig;
struct CorridorParameters {
  double robot_radius;
  std::vector<Eigen::Vector2d> footprint;
  double local_window_width;
  double local_window_height;
};
CorridorParameters CorridorParametersFrom(const NavigationConfig& config);
}  // namespace navigation2d
