#include "navigation2d/control/mpc_controller.h"

#include <cmath>
#include <chrono>
#include <utility>

#include "navigation2d/control/acados_mpc_backend.h"
#include "navigation2d/control/dynamic_safety.h"
#include "navigation2d/control/mppi_controller.h"
#include "navigation2d/control/regulated_pure_pursuit.h"

namespace navigation2d {
namespace {

double NormalizeAngle(double value) { return std::atan2(std::sin(value), std::cos(value)); }

Pose2d Integrate(const Pose2d& pose, const Twist2d& control, double dt) {
  const double yaw = Yaw(pose);
  const double next_yaw = NormalizeAngle(yaw + control.angular * dt);
  if (std::abs(control.angular) < 1e-9)
    return MakePose2d(X(pose) + control.linear * std::cos(yaw) * dt,
                      Y(pose) + control.linear * std::sin(yaw) * dt, next_yaw);
  const double radius = control.linear / control.angular;
  return MakePose2d(X(pose) + radius * (std::sin(next_yaw) - std::sin(yaw)),
                    Y(pose) - radius * (std::cos(next_yaw) - std::cos(yaw)), next_yaw);
}

bool IsStop(const Twist2d& command) {
  return std::abs(command.linear) < 1e-9 && std::abs(command.angular) < 1e-9;
}

}  // namespace

MpcController::MpcController(NavigationConfig config)
    : config_(std::move(config)),
      acados_(std::make_unique<AcadosMpcBackend>(config_)),
      mppi_(std::make_unique<MppiController>(config_)),
      rpp_(std::make_unique<RegulatedPurePursuit>(config_)) {}
MpcController::~MpcController() = default;

Twist2d MpcController::Compute(const Path& path, const Pose2d& pose, Twist2d current,
                               const LayeredCostmap& costmap,
                               const std::vector<PredictedObstacle>& dynamic_obstacles) const {
  const auto started = std::chrono::steady_clock::now();
  diagnostics_ = {};
  const auto finish = [&](ControllerBackend backend, ControllerSolveStatus status,
                          int fallback_level, Twist2d command) {
    diagnostics_.backend = backend;
    diagnostics_.status = status;
    diagnostics_.fallback_level = fallback_level;
    diagnostics_.solve_us = 1e6 * std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    return command;
  };
  if (path.size() < 2)
    return finish(ControllerBackend::kNone, ControllerSolveStatus::kFailure, 3, {});
  if (config_.mpc_solver == "acados") {
    const auto command = acados_->Solve(path, pose, current, dynamic_obstacles);
    diagnostics_ = acados_->Diagnostics();
    if (command && !diagnostics_.deadline_miss &&
        !CollisionImminent(pose, *command, costmap) &&
        !DynamicCollisionImminent(pose, *command, dynamic_obstacles, config_))
      return finish(ControllerBackend::kAcados, ControllerSolveStatus::kSuccess, 0, *command);
  }

  const Twist2d sampled = mppi_->Compute(path, pose, current, costmap, dynamic_obstacles);
  if (!IsStop(sampled) && !CollisionImminent(pose, sampled, costmap) &&
      !DynamicCollisionImminent(pose, sampled, dynamic_obstacles, config_))
    return finish(ControllerBackend::kMppi, ControllerSolveStatus::kSuccess, 1, sampled);

  const Twist2d regulated = rpp_->Compute(path, pose, current, costmap, dynamic_obstacles);
  if (!IsStop(regulated) && !CollisionImminent(pose, regulated, costmap) &&
      !DynamicCollisionImminent(pose, regulated, dynamic_obstacles, config_))
    return finish(ControllerBackend::kRpp, ControllerSolveStatus::kSuccess, 2, regulated);
  return finish(ControllerBackend::kRpp, ControllerSolveStatus::kUnsafe, 3, {});
}

bool MpcController::CollisionImminent(const Pose2d& pose, Twist2d command,
                                      const LayeredCostmap& costmap) const {
  Pose2d projected = pose;
  for (double time = 0.; time < config_.collision_horizon; time += config_.control_period) {
    projected = Integrate(projected, command, config_.control_period);
    if (costmap.lethal(X(projected), Y(projected), config_.robot_radius)) return true;
  }
  return false;
}

}  // namespace navigation2d
