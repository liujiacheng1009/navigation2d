#include "navigation2d/control/mpc_controller.h"
#include "navigation2d/control/acados_mpc_backend.h"
#include "navigation2d/control/dynamic_safety.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

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

std::size_t NearestPathPoint(const Path& path, const Pose2d& pose, std::size_t first) {
  std::size_t nearest = first;
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = first; index < path.size(); ++index) {
    const double candidate = (path[index].translation() - pose.translation()).squaredNorm();
    if (candidate < distance) { distance = candidate; nearest = index; }
  }
  return nearest;
}

double PathHeading(const Path& path, std::size_t index) {
  const std::size_t next = std::min(index + 1, path.size() - 1);
  const std::size_t previous = next == index ? index - 1 : index;
  const auto delta = path[next].translation() - path[previous].translation();
  return std::atan2(delta.y(), delta.x());
}

double Curvature(const Path& path, std::size_t index) {
  if (index == 0 || index + 1 >= path.size()) return 0.;
  const auto first = path[index].translation() - path[index - 1].translation();
  const auto second = path[index + 1].translation() - path[index].translation();
  const double a = first.norm(), b = second.norm();
  const double c = (path[index + 1].translation() - path[index - 1].translation()).norm();
  if (a < 1e-6 || b < 1e-6 || c < 1e-6) return 0.;
  return 2. * std::abs(first.x() * second.y() - first.y() * second.x()) / (a * b * c);
}

struct Node {
  Pose2d pose;
  Twist2d control;
  std::size_t progress = 0;
  int topology = 0;  // -1 left, 0 direct, +1 right initial homotopy seed.
  double cost = 0.;
  std::vector<Twist2d> controls;
};

}  // namespace

MpcController::MpcController(NavigationConfig config)
    : config_(std::move(config)), acados_(std::make_unique<AcadosMpcBackend>(config_)) {}
MpcController::~MpcController() = default;

Twist2d MpcController::Compute(const Path& path, const Pose2d& pose, Twist2d current,
                               const LayeredCostmap& costmap,
                               const std::vector<PredictedObstacle>& dynamic_obstacles) const {
  if (path.size() < 2) return {};
  if (config_.mpc_solver == "acados") {
    const auto command = acados_->Solve(path, pose, current, dynamic_obstacles);
    if (command && !CollisionImminent(pose, *command, costmap) &&
        !DynamicCollisionImminent(pose, *command, dynamic_obstacles, config_))
      return *command;
    // Solver unavailable, failed, or violated the independent safety filter:
    // continue through the portable constrained shooting backend.
  }
  const std::size_t initial_progress = NearestPathPoint(path, pose, 0);
  const double dt = config_.control_period;
  const std::array<double, 3> linear_accels{-config_.max_linear_acceleration, 0., config_.max_linear_acceleration};
  const std::array<double, 5> angular_accels{-config_.max_angular_acceleration,
      -.5 * config_.max_angular_acceleration, 0., .5 * config_.max_angular_acceleration,
      config_.max_angular_acceleration};
  std::vector<Node> beam;
  // These three first-command biases are deliberate multi-topology initial
  // guesses. Subsequent expansion keeps all feasible left/direct/right modes.
  for (int topology : {-1, 0, 1})
    beam.push_back({pose, current, initial_progress, topology, 0., {}});

  for (int step = 0; step < config_.mpc_time_steps; ++step) {
    std::vector<Node> expanded;
    for (const Node& node : beam) for (const double av : linear_accels) for (const double aw : angular_accels) {
      Twist2d control{
          std::clamp(node.control.linear + av * dt, -config_.max_reverse_velocity,
                     config_.desired_linear_velocity),
          std::clamp(node.control.angular + aw * dt, -config_.max_angular_velocity,
                     config_.max_angular_velocity)};
      if (step == 0 && node.topology != 0 && control.angular * node.topology < 0.) continue;
      const Pose2d projected = Integrate(node.pose, control, dt);
      if (costmap.lethal(X(projected), Y(projected), config_.robot_radius)) continue;
      if (DynamicCollisionAt(projected, (step + 1) * dt, dynamic_obstacles, config_)) continue;
      const std::size_t progress = NearestPathPoint(path, projected, node.progress);
      const auto error = projected.translation() - path[progress].translation();
      const double heading_error = NormalizeAngle(PathHeading(path, progress) - Yaw(projected));
      const double curvature = Curvature(path, progress);
      const double curvature_speed = curvature < 1e-6 ? config_.desired_linear_velocity :
          std::sqrt(config_.mpc_max_lateral_acceleration / curvature);
      const double desired_speed = std::min(config_.desired_linear_velocity, curvature_speed);
      const double obstacle_cost = static_cast<double>(costmap.cost(
          costmap.grid().ToCell(X(projected), Y(projected)).first,
          costmap.grid().ToCell(X(projected), Y(projected)).second)) / 254.;
      Node next{projected, control, progress, node.topology,
          node.cost + config_.mpc_contour_weight * error.squaredNorm() +
              config_.mpc_heading_weight * heading_error * heading_error +
              config_.mpc_speed_weight * std::pow(control.linear - desired_speed, 2) +
              config_.mpc_control_weight * (control.linear * control.linear + control.angular * control.angular) +
              config_.mpc_control_rate_weight * (av * av + aw * aw) +
              config_.mpc_obstacle_weight * obstacle_cost * obstacle_cost -
              config_.mpc_progress_weight * static_cast<double>(progress - node.progress),
          node.controls};
      next.controls.push_back(control);
      expanded.push_back(std::move(next));
    }
    if (expanded.empty()) return {};
    std::sort(expanded.begin(), expanded.end(), [](const Node& first, const Node& second) {
      return first.cost < second.cost;
    });
    if (expanded.size() > static_cast<std::size_t>(config_.mpc_beam_width))
      expanded.resize(static_cast<std::size_t>(config_.mpc_beam_width));
    beam = std::move(expanded);
  }
  const Node& best = *std::min_element(beam.begin(), beam.end(), [](const Node& first, const Node& second) {
    return first.cost < second.cost;
  });
  const Twist2d command = best.controls.front();
  return CollisionImminent(pose, command, costmap) ? Twist2d{} : command;
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
