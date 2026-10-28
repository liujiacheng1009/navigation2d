#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <utility>
#include <vector>

#include "navigation2d/control/obstacle_prediction.h"
#include "navigation2d/control/safe_corridor.h"
namespace navigation2d::control_internal {
struct AcadosStageReference {
  Eigen::Vector2d position;
  double heading, velocity, time;
};
struct AcadosObstacleParameters {
  double robot_radius, dynamic_prediction_timeout;
  double mpc_dynamic_safety_margin, mpc_dynamic_sigma_scale;
};
// Pure generated-solver layout adapter; all lengths are metres, time seconds,
// pose and obstacles share one map frame. Does not touch solver state.
// Layout: [x,y,yaw,v], obstacle slots [x,y,rx,ry,enabled], then eight
// corridor slots [nx,ny,bound,enabled]. Model dimension is checked at compile
// time; unused obstacle slots stay far away and unused halfspaces disabled.
template <std::size_t ParameterCount>
std::array<double, ParameterCount> BuildAcadosStageParameters(
    const AcadosStageReference& reference, std::span<const PredictedObstacle> obstacles,
    std::span<const Halfspace2d> halfspaces, const AcadosObstacleParameters& config) {
  constexpr int corridor_slots = 8;
  static_assert(ParameterCount >= 4 + 4 * corridor_slots + 5);
  static_assert((ParameterCount - 4 - 4 * corridor_slots) % 5 == 0);
  constexpr int obstacle_slots = (ParameterCount - 4 - 4 * corridor_slots) / 5;
  std::array<double, ParameterCount> parameters{};
  parameters[0] = reference.position.x();
  parameters[1] = reference.position.y();
  parameters[2] = reference.heading;
  parameters[3] = reference.velocity;
  for (int slot = 0; slot < obstacle_slots; ++slot) {
    const int offset = 4 + slot * 5;
    parameters[offset] = 1e6;
    parameters[offset + 1] = 1e6;
    parameters[offset + 2] = 1.;
    parameters[offset + 3] = 1.;
    parameters[offset + 4] = 0.;
  }
  if (!obstacles.empty()) {
    const double time = reference.time;
    std::vector<const PredictedObstacle*> ranked;
    ranked.reserve(obstacles.size());
    for (const auto& obstacle : obstacles) {
      if (obstacle.age_s >= 0. && obstacle.age_s <= config.dynamic_prediction_timeout) {
        ranked.push_back(&obstacle);
      }
    }
    std::sort(ranked.begin(), ranked.end(), [&](const auto* first, const auto* second) {
      const auto risk = [&](const PredictedObstacle* obstacle) {
        const double prediction_time = time + obstacle->age_s;
        const Eigen::Vector2d relative(
            obstacle->x + obstacle->vx * prediction_time - reference.position.x(),
            obstacle->y + obstacle->vy * prediction_time - reference.position.y());
        const Eigen::Vector2d path_velocity(reference.velocity * std::cos(reference.heading),
                                            reference.velocity * std::sin(reference.heading));
        const Eigen::Vector2d relative_velocity(obstacle->vx - path_velocity.x(),
                                                obstacle->vy - path_velocity.y());
        const double speed_squared = relative_velocity.squaredNorm();
        const double ttc =
            speed_squared < 1e-8
                ? 1e6
                : std::clamp(-relative.dot(relative_velocity) / speed_squared, 0., 10.);
        const double closest = (relative + ttc * relative_velocity).norm();
        return std::pair<double, double>{closest, ttc};
      };
      return risk(first) < risk(second);
    });
    const int active_slots = std::min(obstacle_slots, static_cast<int>(ranked.size()));
    for (int slot = 0; slot < active_slots; ++slot) {
      const auto* obstacle = ranked[slot];
      const double radius =
          config.robot_radius + obstacle->radius + config.mpc_dynamic_safety_margin;
      const int offset = 4 + slot * 5;
      const double prediction_time = time + obstacle->age_s;
      parameters[offset] = obstacle->x + obstacle->vx * prediction_time;
      parameters[offset + 1] = obstacle->y + obstacle->vy * prediction_time;
      parameters[offset + 2] = radius + config.mpc_dynamic_sigma_scale * obstacle->sigma_x;
      parameters[offset + 3] = radius + config.mpc_dynamic_sigma_scale * obstacle->sigma_y;
      parameters[offset + 4] = 1.;
    }
  }
  const int corridor_offset = 4 + obstacle_slots * 5;
  for (int slot = 0; slot < corridor_slots; ++slot) {
    const int offset = corridor_offset + slot * 4;
    parameters[offset] = 0.;
    parameters[offset + 1] = 0.;
    parameters[offset + 2] = 0.;
    parameters[offset + 3] = 0.;
  }
  if (!halfspaces.empty()) {
    const int active = std::min(corridor_slots, static_cast<int>(halfspaces.size()));
    for (int slot = 0; slot < active; ++slot) {
      const int offset = corridor_offset + slot * 4;
      parameters[offset] = halfspaces[static_cast<std::size_t>(slot)].normal.x();
      parameters[offset + 1] = halfspaces[static_cast<std::size_t>(slot)].normal.y();
      parameters[offset + 2] = halfspaces[static_cast<std::size_t>(slot)].bound;
      parameters[offset + 3] = 1.;
    }
  }
  return parameters;
}
}  // namespace navigation2d::control_internal
