#include "navigation2d/control/mpc_controller.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <future>
#include <limits>
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

bool MakesProgress(const Path& path, const Pose2d& pose, const Twist2d& command) {
  if ((path.back().translation() - pose.translation()).norm() < .3) return true;
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double distance = (path[index].translation() - pose.translation()).squaredNorm();
    if (distance < best) { best = distance; nearest = index; }
  }
  const auto target = path[std::min(nearest + 4, path.size() - 1)].translation();
  const auto delta = target - pose.translation();
  const double heading_error = NormalizeAngle(std::atan2(delta.y(), delta.x()) - Yaw(pose));
  // While misaligned, only rotation toward the current path is productive.
  // Once aligned, require positive velocity along it; stale MPC warm starts
  // must not be accepted merely because they produce a non-zero command.
  if (std::abs(heading_error) > .35)
    return command.angular * heading_error > 0. && std::abs(command.angular) >= .03;
  return command.linear * std::cos(heading_error) >= .04;
}

}  // namespace

MpcController::MpcController(NavigationConfig config)
    : config_(std::move(config)),
      acados_(std::make_unique<AcadosMpcBackend>(config_)),
      mppi_(std::make_unique<MppiController>(config_)),
      rpp_(std::make_unique<RegulatedPurePursuit>(config_)) {
  for (int index = 1; index < config_.guidance_max_candidates; ++index)
    additional_acados_.push_back(std::make_unique<AcadosMpcBackend>(config_));
}
MpcController::~MpcController() = default;

void MpcController::SetGuidanceCandidates(std::vector<GuidanceCandidate> candidates) {
  guidance_candidates_ = std::move(candidates);
}

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
  if (config_.mpc_solver == "acados" && acados_->available()) {
    std::vector<const Path*> candidates;
    for (const auto& candidate : guidance_candidates_) {
      if (candidate.age_s < 0. || candidate.age_s > config_.guidance_candidate_timeout ||
          candidate.path.size() < 2) continue;
      candidates.push_back(&candidate.path);
      if (candidates.size() + 1 >=
          static_cast<std::size_t>(config_.guidance_max_candidates)) break;
    }
    // TUD Guidance candidates arrive in deterministic quality order. Preserve
    // that order and retain the unmodified global path as the final fallback.
    candidates.push_back(&path);
    struct SolveResult {
      std::optional<Twist2d> command;
      ControllerDiagnostics diagnostics;
    };
    const auto solve_candidate = [&](std::size_t index) {
      AcadosMpcBackend* backend = index == 0 ? acados_.get() :
          additional_acados_[index - 1].get();
      const Path* candidate = candidates[index];
      const auto command = backend->Solve(
          *candidate, pose, current, costmap, dynamic_obstacles);
      return SolveResult{command, backend->Diagnostics()};
    };
    std::vector<SolveResult> results;
    results.reserve(candidates.size());
    if (candidates.size() == 1) {
      // Avoid thread creation and scheduling jitter on the normal, single-path
      // control cycle. Parallel work is reserved for actual multi-topology input.
      results.push_back(solve_candidate(0));
    } else {
      std::vector<std::future<SolveResult>> futures;
      futures.reserve(candidates.size());
      for (std::size_t index = 0; index < candidates.size(); ++index) {
        futures.push_back(std::async(std::launch::async,
            [&, index]() { return solve_candidate(index); }));
      }
      for (auto& future : futures) results.push_back(future.get());
    }
    bool any_deadline_miss = false;
    for (const auto& result : results) {
      any_deadline_miss = any_deadline_miss || result.diagnostics.deadline_miss;
      if (result.command && !result.diagnostics.deadline_miss &&
          MakesProgress(path, pose, *result.command) &&
          !CollisionImminent(pose, *result.command, costmap) &&
          !DynamicCollisionImminent(pose, *result.command, dynamic_obstacles, config_)) {
        diagnostics_ = result.diagnostics;
        return finish(ControllerBackend::kAcados, ControllerSolveStatus::kSuccess,
                      0, *result.command);
      }
    }
    diagnostics_.deadline_miss = any_deadline_miss;
  }

  const Twist2d sampled = mppi_->Compute(path, pose, current, costmap, dynamic_obstacles);
  if (!IsStop(sampled) && MakesProgress(path, pose, sampled) &&
      !CollisionImminent(pose, sampled, costmap) &&
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
  // The global planner/path validator owns the rasterization margin.  Keep
  // the local controller's swept check on the declared physical footprint so
  // a legal narrow-gate turn is not rejected twice.
  const double collision_radius = config_.robot_radius;
  Pose2d projected = pose;
  for (double time = 0.; time < config_.collision_horizon; time += config_.control_period) {
    projected = Integrate(projected, command, config_.control_period);
    if (costmap.lethal(X(projected), Y(projected), collision_radius)) return true;
  }
  return false;
}

}  // namespace navigation2d
