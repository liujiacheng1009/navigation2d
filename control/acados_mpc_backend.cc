#include "navigation2d/control/acados_mpc_backend.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

extern "C" {
#include "acados_solver_navigation2d_mpcc.h"
}

namespace navigation2d {
namespace {
std::size_t NearestPathPoint(const Path& path, const Pose2d& pose) {
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double distance = (path[index].translation() - pose.translation()).squaredNorm();
    if (distance < best) { best = distance; nearest = index; }
  }
  return nearest;
}

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
};

AcadosMpcBackend::AcadosMpcBackend(const NavigationConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}
AcadosMpcBackend::~AcadosMpcBackend() = default;
bool AcadosMpcBackend::available() const { return impl_->ready; }

std::optional<Twist2d> AcadosMpcBackend::Solve(
    const Path& path, const Pose2d& pose, Twist2d current,
    const std::vector<PredictedObstacle>& obstacles) const {
  if (!impl_->ready || path.size() < 2) return std::nullopt;
  auto* capsule = impl_->capsule;
  auto* config = navigation2d_mpcc_acados_get_nlp_config(capsule);
  auto* dims = navigation2d_mpcc_acados_get_nlp_dims(capsule);
  auto* input = navigation2d_mpcc_acados_get_nlp_in(capsule);
  auto* output = navigation2d_mpcc_acados_get_nlp_out(capsule);
  std::array<double, 5> state{X(pose), Y(pose), Yaw(pose), current.linear, current.angular};
  ocp_nlp_constraints_model_set(config, dims, input, output, 0, "lbx", state.data());
  ocp_nlp_constraints_model_set(config, dims, input, output, 0, "ubx", state.data());

  const std::size_t nearest = NearestPathPoint(path, pose);
  const double robot_remaining = (path.back().translation() - pose.translation()).norm();
  constexpr int horizon = NAVIGATION2D_MPCC_N;
  for (int stage = 0; stage <= horizon; ++stage) {
    const bool approach_goal = robot_remaining < .5;
    const std::size_t reference = approach_goal ? path.size() - 1 : AdvancePath(
        path, nearest, impl_->config.desired_linear_velocity * stage * impl_->config.control_period);
    const double reference_heading = approach_goal
        ? std::atan2(Y(path.back()) - Y(pose), X(path.back()) - X(pose))
        : PathHeading(path, reference);
    const double reference_velocity = std::min(
        impl_->config.desired_linear_velocity,
        std::max(impl_->config.min_approach_velocity, robot_remaining * .8));
    static_assert(NAVIGATION2D_MPCC_NP >= 9 &&
                  (NAVIGATION2D_MPCC_NP - 4) % 5 == 0);
    constexpr int obstacle_slots = (NAVIGATION2D_MPCC_NP - 4) / 5;
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
        ranked.push_back(&obstacle);
      }
      std::sort(ranked.begin(), ranked.end(), [&](const auto* first, const auto* second) {
        const auto distance = [&](const PredictedObstacle* obstacle) {
          return std::hypot(X(path[reference]) - obstacle->x - obstacle->vx * time,
                            Y(path[reference]) - obstacle->y - obstacle->vy * time);
        };
        return distance(first) < distance(second);
      });
      const int active_slots = std::min(obstacle_slots, static_cast<int>(ranked.size()));
      for (int slot = 0; slot < active_slots; ++slot) {
        const auto* obstacle = ranked[slot];
        const double radius = impl_->config.robot_radius + obstacle->radius +
                              impl_->config.mpc_dynamic_safety_margin;
        const int offset = 4 + slot * 5;
        parameters[offset] = obstacle->x + obstacle->vx * time;
        parameters[offset + 1] = obstacle->y + obstacle->vy * time;
        parameters[offset + 2] = radius + impl_->config.mpc_dynamic_sigma_scale * obstacle->sigma_x;
        parameters[offset + 3] = radius + impl_->config.mpc_dynamic_sigma_scale * obstacle->sigma_y;
        parameters[offset + 4] = 1.;
      }
    }
    if (navigation2d_mpcc_acados_update_params(capsule, stage, parameters.data(),
                                                static_cast<int>(parameters.size())) != 0)
      return std::nullopt;
    ocp_nlp_out_set(config, dims, output, input, stage, "x", state.data());
  }
  if (navigation2d_mpcc_acados_solve(capsule) != 0) return std::nullopt;
  std::array<double, 2> acceleration{};
  ocp_nlp_out_get(config, dims, output, 0, "u", acceleration.data());
  if (!std::isfinite(acceleration[0]) || !std::isfinite(acceleration[1])) return std::nullopt;
  return Twist2d{
      std::clamp(current.linear + acceleration[0] * impl_->config.control_period,
                 -impl_->config.max_reverse_velocity, impl_->config.desired_linear_velocity),
      std::clamp(current.angular + acceleration[1] * impl_->config.control_period,
                 -impl_->config.max_angular_velocity, impl_->config.max_angular_velocity)};
}

}  // namespace navigation2d
