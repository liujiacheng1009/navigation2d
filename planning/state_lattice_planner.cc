#include "navigation2d/planning/state_lattice_planner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "navigation2d/planning/collision_checker.h"
#include "navigation2d/planning/motion_primitives.h"
#include "navigation2d/planning/search_core.h"

namespace navigation2d {
namespace {
int YawBin(double yaw, int bins) {
  const double normalized = std::fmod(std::fmod(yaw, 2. * M_PI) + 2. * M_PI, 2. * M_PI);
  return static_cast<int>(std::lround(normalized * bins / (2. * M_PI))) % bins;
}

double AngleDifference(double first, double second) {
  return std::abs(std::atan2(std::sin(first - second), std::cos(first - second)));
}
}  // namespace

Path StateLatticePlanner::Plan(const LayeredCostmap& costmap, const Pose2d& start,
                               const Pose2d& goal) const {
  const Grid2d& grid = costmap.grid();
  const int bins = config_.lattice_yaw_bins;
  const auto [sx, sy] = grid.ToCell(X(start), Y(start));
  const auto [gx, gy] = grid.ToCell(X(goal), Y(goal));
  planning_internal::DistanceField distance_field(costmap);
  if (!distance_field.CircleCollisionFree(X(start), Y(start), config_.robot_radius) ||
      !distance_field.CircleCollisionFree(X(goal), Y(goal), config_.robot_radius))
    throw std::runtime_error("start or goal is occupied");
  const planning_internal::DifferentialDrivePrimitiveSet primitives(
      bins, grid.resolution(), config_.lattice_primitive_length,
      config_.lattice_allow_reverse);
  const auto encode = [&](int x, int y, int yaw_bin) {
    return (y * grid.width() + x) * bins + yaw_bin;
  };
  const auto decode = [&](int state, int* x, int* y, int* yaw_bin) {
    *yaw_bin = state % bins;
    const int cell = state / bins;
    *x = cell % grid.width();
    *y = cell / grid.width();
  };
  const int start_bin = YawBin(Yaw(start), bins), goal_bin = YawBin(Yaw(goal), bins);
  const int source = encode(sx, sy, start_bin), target = encode(gx, gy, goal_bin);
  const bool cache_hit = obstacle_heuristic_ && obstacle_heuristic_->Matches(
      costmap, gx, gy, config_.robot_radius, config_.lattice_cost_penalty);
  if (!cache_hit)
    obstacle_heuristic_.emplace(costmap, gx, gy, config_.robot_radius,
                                config_.lattice_cost_penalty);
  planning_internal::SearchOptions options;
  options.max_expansions = static_cast<std::size_t>(config_.lattice_max_expansions);
  options.max_planning_time_s = config_.lattice_max_planning_time;
  const double bin_angle = 2. * M_PI / bins;
  const auto heuristic = [&](int state) {
        int x, y, yaw_bin;
        decode(state, &x, &y, &yaw_bin);
        const double distance = std::hypot(gx - x, gy - y) * grid.resolution();
        const double yaw = yaw_bin * bin_angle;
        const double kinematic = std::max(
            distance, config_.lattice_rotation_cost * AngleDifference(yaw, Yaw(goal)));
        return std::max(kinematic, obstacle_heuristic_->cost(x, y));
      };
  const auto expand =
      [&](int state, int, const planning_internal::SuccessorVisitor& visit) {
        int x, y, yaw_bin;
        decode(state, &x, &y, &yaw_bin);
        const auto [wx, wy] = grid.CellCenter(x, y);
        for (const auto& primitive : primitives.FromYawBin(yaw_bin)) {
          const int nx = x + primitive.dx_cells, ny = y + primitive.dy_cells;
          if (nx < 0 || ny < 0 || nx >= grid.width() || ny >= grid.height()) continue;
          bool collision_free = true;
          double cost_integral = 0.;
          for (const auto& sample : primitive.samples) {
            const double px = wx + X(sample), py = wy + Y(sample);
            if (!distance_field.CircleCollisionFree(px, py, config_.robot_radius)) {
              collision_free = false;
              break;
            }
            const auto [cx, cy] = grid.ToCell(px, py);
            cost_integral += static_cast<double>(costmap.cost(cx, cy)) / 252.;
          }
          if (!collision_free) continue;
          double transition = primitive.type == planning_internal::MotionType::kRotate
              ? config_.lattice_rotation_cost * std::abs(primitive.yaw_change)
              : primitive.length_m;
          if (primitive.type == planning_internal::MotionType::kReverse)
            transition *= config_.lattice_reverse_penalty;
          transition += config_.lattice_cost_penalty * primitive.length_m *
                        cost_integral / primitive.samples.size();
          visit({encode(nx, ny, primitive.end_yaw_bin), transition});
        }
      };
  const auto result = planning_internal::AnytimeRepairingAStar(
      grid.width() * grid.height() * bins, source, target, heuristic, expand,
      config_.lattice_initial_heuristic_weight,
      config_.lattice_heuristic_weight_decrement, options);
  diagnostics_ = {result.diagnostics.expansions, result.diagnostics.generated,
                  result.diagnostics.elapsed_s, result.diagnostics.first_solution_s,
                  result.suboptimality_bound, cache_hit};
  if (!result.found) {
    if (result.diagnostics.time_limit_reached) throw std::runtime_error("state lattice planning timed out");
    if (result.diagnostics.expansion_limit_reached) throw std::runtime_error("state lattice expansion limit reached");
    throw std::runtime_error("no state lattice path");
  }

  const auto states = planning_internal::RestoreStatePath(result, source, target);
  Path path{start};
  for (std::size_t index = 1; index < states.size(); ++index) {
    int x0, y0, yaw0, x1, y1, yaw1;
    decode(states[index - 1], &x0, &y0, &yaw0);
    decode(states[index], &x1, &y1, &yaw1);
    const auto* primitive = primitives.Find(yaw0, x1 - x0, y1 - y0, yaw1);
    if (primitive == nullptr) throw std::runtime_error("missing lattice parent primitive");
    const auto [wx, wy] = grid.CellCenter(x0, y0);
    for (const auto& sample : primitive->samples)
      path.push_back(MakePose2d(wx + X(sample), wy + Y(sample), Yaw(sample)));
  }
  path.back() = goal;
  return path;
}

}  // namespace navigation2d
