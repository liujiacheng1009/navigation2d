#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
namespace navigation2d::exploration {
enum class MissionPhase { kExploring, kCompleting, kReturning };
struct MissionHome {
  double x = 0., y = 0., yaw = 0.;  // Map frame, metres/radians.
};
struct MissionPosition {
  double x = 0., y = 0.;
};
struct MissionCompletion {
  bool success = false;
  std::optional<double> home_error_m;
  std::string message;
};
}  // namespace navigation2d::exploration
