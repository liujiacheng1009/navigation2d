#include "navigation2d/control/acados_mpc_backend.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

#include "navigation2d/control/path_tracking.h"
#include "navigation2d/control/safe_corridor.h"

extern "C" {
#include "acados_solver_navigation2d_mpcc.h"
}

namespace navigation2d {
namespace {
double PathHeading(const Path& path, std::size_t index) {
  if (path.size() < 2) return Yaw(path.back());
  const std::size_t next = std::min(index + 1, path.size() - 1);
  const std::size_t previous = next == index ? index - 1 : index;
  const auto delta = path[next].translation() - path[previous].translation();
  return std::atan2(delta.y(), delta.x());
}

std::size_t AdvancePath(const Path& path, std::size_t start, double distance) {
  double accumulated = 0.;
  std::size_t index = start;
  while (index + 1 < path.size() && accumulated < distance) {
    accumulated += (path[index + 1].translation() - path[index].translation()).norm();
    ++index;
  }
  return index;
}
}

struct AcadosMpcBackend::Impl {
  explicit Impl(const NavigationConfig& value) : config(value) {
    capsule = navigation2d_mpcc_acados_create_capsule();
    ready = capsule != nullptr && navigation2d_mpcc_acados_create(capsule) == 0;
  }
  ~Impl() {
    if (capsule != nullptr) {
      if (ready) navigation2d_mpcc_acados_free(capsule);
      navigation2d_mpcc_acados_free_capsule(capsule);
    }
  }
  NavigationConfig config;
  bool ready = false;
  navigation2d_mpcc_solver_capsule* capsule = nullptr;
  bool has_warm_start = false;
  control_internal::PathSearchState path_search;
  std::array<std::array<double, NAVIGATION2D_MPCC_NX>, NAVIGATION2D_MPCC_N + 1> states{};
  std::array<std::array<double, NAVIGATION2D_MPCC_NU>, NAVIGATION2D_MPCC_N> controls{};
  ControllerDiagnostics diagnostics;
};

AcadosMpcBackend::AcadosMpcBackend(const NavigationConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}
AcadosMpcBackend::~AcadosMpcBackend() = default;
bool AcadosMpcBackend::available() const { return impl_->ready; }
ControllerDiagnostics AcadosMpcBackend::Diagnostics() const { return impl_->diagnostics; }

std::optional<Twist2d> AcadosMpcBackend::Solve(
    const Path& path, const Pose2d& pose, Twist2d current,
    const LayeredCostmap& costmap,
    const std::vector<PredictedObstacle>& obstacles) const {
  impl_->diagnostics = {};
  impl_->diagnostics.backend = ControllerBackend::kAcados;
  if (!impl_->ready || path.size() < 2) {
    impl_->diagnostics.status = ControllerSolveStatus::kUnavailable;
    return std::nullopt;
  }
  const auto solve_started = std::chrono::steady_clock::now();
  auto* capsule = impl_->capsule;
  auto* config = navigation2d_mpcc_acados_get_nlp_config(capsule);
  auto* dims = navigation2d_mpcc_acados_get_nlp_dims(capsule);
  auto* input = navigation2d_mpcc_acados_get_nlp_in(capsule);
  auto* output = navigation2d_mpcc_acados_get_nlp_out(capsule);
  std::array<double, 5> state{X(pose), Y(pose), Yaw(pose), current.linear, current.angular};
  static_assert(NAVIGATION2D_MPCC_NX == 5 && NAVIGATION2D_MPCC_NU == 2);
  ocp_nlp_constraints_model_set(config, dims, input, output, 0, "lbx", state.data());
  ocp_nlp_constraints_model_set(config, dims, input, output, 0, "ubx", state.data());

  const std::size_t nearest =
      control_internal::FindNearestPathPoint(path, pose, &impl_->path_search);
  impl_->diagnostics.path_search_evaluations = impl_->path_search.last_evaluations;
  const double robot_remaining = (path.back().translation() - pose.translation()).norm();
  constexpr int horizon = NAVIGATION2D_MPCC_N;
  std::array<std::size_t, horizon + 1> references{};
  Path corridor_reference;
  corridor_reference.reserve(horizon + 1);
  for (int stage = 0; stage <= horizon; ++stage) {
    references[static_cast<std::size_t>(stage)] = robot_remaining < .5 ? path.size() - 1 :
        AdvancePath(path, nearest,
                    impl_->config.desired_linear_velocity * stage * impl_->config.control_period);
    corridor_reference.push_back(path[references[static_cast<std::size_t>(stage)]]);
  }
  if (robot_remaining < .5) corridor_reference = {pose, path.back()};
  const auto corridor = control_internal::BuildSafeCorridor(
      corridor_reference, costmap, impl_->config);
  if (impl_->has_warm_start) {
    for (int stage = 0; stage <= horizon; ++stage) {
      const int source = std::min(stage + 1, horizon);
      ocp_nlp_out_set(config, dims, output, input, stage, "x",
                      impl_->states[static_cast<std::size_t>(source)].data());
      if (stage < horizon) {
        const int control_source = std::min(stage + 1, horizon - 1);
        ocp_nlp_out_set(config, dims, output, input, stage, "u",
                        impl_->controls[static_cast<std::size_t>(control_source)].data());
      }
    }
  }
  for (int stage = 0; stage <= horizon; ++stage) {
    const bool approach_goal = robot_remaining < .5;
    const std::size_t reference = references[static_cast<std::size_t>(stage)];
    const double reference_heading = approach_goal
        ? std::atan2(Y(path.back()) - Y(pose), X(path.back()) - X(pose))
        : PathHeading(path, reference);
    const double reference_velocity = std::min(
        impl_->config.desired_linear_velocity,
        std::max(impl_->config.min_approach_velocity, robot_remaining * .8));
    constexpr int corridor_slots = 8;
    static_assert(NAVIGATION2D_MPCC_NP >= 4 + 4 * corridor_slots + 5);
    static_assert((NAVIGATION2D_MPCC_NP - 4 - 4 * corridor_slots) % 5 == 0);
    constexpr int obstacle_slots =
        (NAVIGATION2D_MPCC_NP - 4 - 4 * corridor_slots) / 5;
    std::array<double, NAVIGATION2D_MPCC_NP> parameters{};
    parameters[0] = X(path[reference]);
    parameters[1] = Y(path[reference]);
    parameters[2] = reference_heading;
    parameters[3] = reference_velocity;
    for (int slot = 0; slot < obstacle_slots; ++slot) {
      const int offset = 4 + slot * 5;
      parameters[offset] = 1e6;
      parameters[offset + 1] = 1e6;
      parameters[offset + 2] = 1.;
      parameters[offset + 3] = 1.;
      parameters[offset + 4] = 0.;
    }
    if (!obstacles.empty()) {
      const double time = stage * impl_->config.control_period;
      std::vector<const PredictedObstacle*> ranked;
      ranked.reserve(obstacles.size());
      for (const auto& obstacle : obstacles) {
        if (obstacle.age_s >= 0. &&
            obstacle.age_s <= impl_->config.dynamic_prediction_timeout)
          ranked.push_back(&obstacle);
      }
      std::sort(ranked.begin(), ranked.end(), [&](const auto* first, const auto* second) {
        const auto risk = [&](const PredictedObstacle* obstacle) {
          const double prediction_time = time + obstacle->age_s;
          const Eigen::Vector2d relative(
              obstacle->x + obstacle->vx * prediction_time - X(path[reference]),
              obstacle->y + obstacle->vy * prediction_time - Y(path[reference]));
          const Eigen::Vector2d path_velocity(
              reference_velocity * std::cos(reference_heading),
              reference_velocity * std::sin(reference_heading));
          const Eigen::Vector2d relative_velocity(
              obstacle->vx - path_velocity.x(), obstacle->vy - path_velocity.y());
          const double speed_squared = relative_velocity.squaredNorm();
          const double ttc = speed_squared < 1e-8 ? 1e6 :
              std::clamp(-relative.dot(relative_velocity) / speed_squared, 0., 10.);
          const double closest = (relative + ttc * relative_velocity).norm();
          return std::pair<double, double>{closest, ttc};
        };
        return risk(first) < risk(second);
      });
      const int active_slots = std::min(obstacle_slots, static_cast<int>(ranked.size()));
      for (int slot = 0; slot < active_slots; ++slot) {
        const auto* obstacle = ranked[slot];
        const double radius = impl_->config.robot_radius + obstacle->radius +
                              impl_->config.mpc_dynamic_safety_margin;
        const int offset = 4 + slot * 5;
        const double prediction_time = time + obstacle->age_s;
        parameters[offset] = obstacle->x + obstacle->vx * prediction_time;
        parameters[offset + 1] = obstacle->y + obstacle->vy * prediction_time;
        parameters[offset + 2] = radius + impl_->config.mpc_dynamic_sigma_scale * obstacle->sigma_x;
        parameters[offset + 3] = radius + impl_->config.mpc_dynamic_sigma_scale * obstacle->sigma_y;
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
    if (!corridor.empty()) {
      const auto& halfspaces = corridor[std::min(
          static_cast<std::size_t>(stage), corridor.size() - 1)];
      const int active = std::min(corridor_slots, static_cast<int>(halfspaces.size()));
      for (int slot = 0; slot < active; ++slot) {
        const int offset = corridor_offset + slot * 4;
        parameters[offset] = halfspaces[static_cast<std::size_t>(slot)].normal.x();
        parameters[offset + 1] = halfspaces[static_cast<std::size_t>(slot)].normal.y();
        parameters[offset + 2] = halfspaces[static_cast<std::size_t>(slot)].bound;
        parameters[offset + 3] = 1.;
      }
    }
    if (navigation2d_mpcc_acados_update_params(capsule, stage, parameters.data(),
                                                static_cast<int>(parameters.size())) != 0)
      return std::nullopt;
    if (!impl_->has_warm_start)
      ocp_nlp_out_set(config, dims, output, input, stage, "x", state.data());
  }
  const int status = navigation2d_mpcc_acados_solve(capsule);
  auto* solver = navigation2d_mpcc_acados_get_nlp_solver(capsule);
  ocp_nlp_get(solver, "time_tot", &impl_->diagnostics.solver_us);
  impl_->diagnostics.solver_us *= 1e6;
  ocp_nlp_get(solver, "sqp_iter", &impl_->diagnostics.iterations);
  ocp_nlp_get(solver, "nlp_res", &impl_->diagnostics.kkt_residual);
  impl_->diagnostics.solve_us = 1e6 * std::chrono::duration<double>(
      std::chrono::steady_clock::now() - solve_started).count();
  impl_->diagnostics.deadline_miss =
      impl_->diagnostics.solve_us > impl_->config.mpc_deadline * 1e6;
  if (status != 0) {
    impl_->diagnostics.status = ControllerSolveStatus::kFailure;
    impl_->has_warm_start = false;
    return std::nullopt;
  }
  for (int stage = 0; stage <= horizon; ++stage) {
    ocp_nlp_out_get(config, dims, output, stage, "x",
                    impl_->states[static_cast<std::size_t>(stage)].data());
    if (stage < horizon)
      ocp_nlp_out_get(config, dims, output, stage, "u",
                      impl_->controls[static_cast<std::size_t>(stage)].data());
  }
  impl_->has_warm_start = true;
  impl_->diagnostics.status = impl_->diagnostics.deadline_miss
      ? ControllerSolveStatus::kDeadlineMiss : ControllerSolveStatus::kSuccess;
  std::array<double, 2> acceleration{};
  ocp_nlp_out_get(config, dims, output, 0, "u", acceleration.data());
  if (!std::isfinite(acceleration[0]) || !std::isfinite(acceleration[1])) {
    impl_->diagnostics.status = ControllerSolveStatus::kFailure;
    return std::nullopt;
  }
  return Twist2d{
      std::clamp(current.linear + acceleration[0] * impl_->config.control_period,
                 -impl_->config.max_reverse_velocity, impl_->config.desired_linear_velocity),
      std::clamp(current.angular + acceleration[1] * impl_->config.control_period,
                 -impl_->config.max_angular_velocity, impl_->config.max_angular_velocity)};
}

}  // namespace navigation2d
