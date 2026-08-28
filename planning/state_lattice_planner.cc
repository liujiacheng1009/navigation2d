#include "navigation2d/planning/state_lattice_planner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "navigation2d/planning/path_smoother.h"

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

int StateLatticePlanner::Encode(int x, int y, int yaw_bin) const {
  return (y * lattice_width_ + x) * config_.lattice_yaw_bins + yaw_bin;
}

void StateLatticePlanner::Decode(int state, int* x, int* y, int* yaw_bin) const {
  *yaw_bin = state % config_.lattice_yaw_bins;
  const int cell = state / config_.lattice_yaw_bins;
  *x = cell % lattice_width_;
  *y = cell / lattice_width_;
}

std::optional<double> StateLatticePlanner::PrimitiveCost(
    int x, int y, const planning_internal::MotionPrimitive& primitive) const {
  const int nx = x + primitive.dx_cells, ny = y + primitive.dy_cells;
  if (nx < 0 || ny < 0 || nx >= lattice_width_ || ny >= lattice_height_) return std::nullopt;
  const auto [wx, wy] = active_costmap_->grid().CellCenter(x, y);
  double cost_integral = 0.;
  for (const auto& sample : primitive.samples) {
    const double px = wx + X(sample), py = wy + Y(sample);
    if (!distance_field_->CircleCollisionFree(px, py, config_.robot_radius)) return std::nullopt;
    const auto [cx, cy] = active_costmap_->grid().ToCell(px, py);
    cost_integral += static_cast<double>(active_costmap_->cost(cx, cy)) / 252.;
  }
  double transition = primitive.type == planning_internal::MotionType::kRotate
      ? config_.lattice_rotation_cost * std::abs(primitive.yaw_change) : primitive.length_m;
  if (primitive.type == planning_internal::MotionType::kReverse)
    transition *= config_.lattice_reverse_penalty;
  transition += config_.lattice_cost_penalty * primitive.length_m *
                cost_integral / primitive.samples.size();
  return transition;
}

void StateLatticePlanner::VisitSuccessors(
    int state, const planning_internal::SuccessorVisitor& visit) const {
  int x, y, yaw_bin; Decode(state, &x, &y, &yaw_bin);
  for (const auto& primitive : primitives_->FromYawBin(yaw_bin)) {
    const auto cost = PrimitiveCost(x, y, primitive);
    if (cost) visit({Encode(x + primitive.dx_cells, y + primitive.dy_cells,
                            primitive.end_yaw_bin), *cost});
  }
}

void StateLatticePlanner::VisitPredecessors(
    int state, const planning_internal::SuccessorVisitor& visit) const {
  int x, y, yaw_bin; Decode(state, &x, &y, &yaw_bin);
  for (const auto& reference : primitives_->IntoYawBin(yaw_bin)) {
    const auto& primitive = primitives_->FromYawBin(reference.start_yaw_bin)[reference.index];
    const int px = x - primitive.dx_cells, py = y - primitive.dy_cells;
    if (px < 0 || py < 0 || px >= lattice_width_ || py >= lattice_height_) continue;
    const auto cost = PrimitiveCost(px, py, primitive);
    if (cost) visit({Encode(px, py, reference.start_yaw_bin), *cost});
  }
}

double StateLatticePlanner::PairHeuristic(int first, int second) const {
  int first_x, first_y, first_yaw, second_x, second_y, second_yaw;
  Decode(first, &first_x, &first_y, &first_yaw);
  Decode(second, &second_x, &second_y, &second_yaw);
  const double distance = std::hypot(first_x - second_x, first_y - second_y) *
                          active_costmap_->grid().resolution();
  const double bin_angle = 2. * M_PI / config_.lattice_yaw_bins;
  return std::max(distance, config_.lattice_rotation_cost *
      AngleDifference(first_yaw * bin_angle, second_yaw * bin_angle));
}

Path StateLatticePlanner::Plan(const LayeredCostmap& costmap, const Pose2d& start,
                               const Pose2d& goal) const {
  const Grid2d& grid = costmap.grid();
  const int bins = config_.lattice_yaw_bins;
  const auto [sx, sy] = grid.ToCell(X(start), Y(start));
  const auto [gx, gy] = grid.ToCell(X(goal), Y(goal));
  const bool different_map = active_costmap_ != nullptr && active_costmap_ != &costmap;
  const std::uint64_t previous_revision = map_revision_;
  active_costmap_ = &costmap;
  lattice_width_ = grid.width();
  lattice_height_ = grid.height();
  if (!primitives_ || different_map)
    primitives_.emplace(bins, grid.resolution(), config_.lattice_primitive_length,
                        config_.lattice_allow_reverse);
  if (!distance_field_ || different_map || previous_revision != costmap.revision())
    distance_field_ = std::make_unique<planning_internal::DistanceField>(costmap);
  if (!distance_field_->CircleCollisionFree(X(start), Y(start), config_.robot_radius) ||
      !distance_field_->CircleCollisionFree(X(goal), Y(goal), config_.robot_radius))
    throw std::runtime_error("start or goal is occupied");

  const int source = Encode(sx, sy, YawBin(Yaw(start), bins));
  const int target = Encode(gx, gy, YawBin(Yaw(goal), bins));
  const bool cache_hit = obstacle_heuristic_ && obstacle_heuristic_->Matches(
      costmap, gx, gy, config_.robot_radius, config_.lattice_cost_penalty);

  std::vector<int> changed_states;
  bool reset_incremental = different_map || !incremental_search_ || incremental_goal_ != target;
  const auto changed_cells = reset_incremental ? std::vector<int>{} :
      costmap.ChangedCellsSince(previous_revision);
  if (!reset_incremental && changed_cells.size() >
      static_cast<std::size_t>(grid.width() * grid.height() / 4)) reset_incremental = true;
  if (reset_incremental) {
    incremental_goal_ = target;
    incremental_search_ = std::make_unique<planning_internal::DStarLite>(
        grid.width() * grid.height() * bins, target,
        [this](int state, const auto& visit) { VisitSuccessors(state, visit); },
        [this](int state, const auto& visit) { VisitPredecessors(state, visit); },
        [this](int first, int second) { return PairHeuristic(first, second); });
  } else if (!changed_cells.empty()) {
    std::vector<bool> affected(grid.width() * grid.height() * bins, false);
    for (int cell : changed_cells) {
      const int cx = cell % grid.width(), cy = cell / grid.width();
      for (int yaw = 0; yaw < bins; ++yaw)
        for (const auto& primitive : primitives_->FromYawBin(yaw))
          for (const auto& sample : primitive.samples) {
            const int offset_x = static_cast<int>(std::lround(X(sample) / grid.resolution()));
            const int offset_y = static_cast<int>(std::lround(Y(sample) / grid.resolution()));
            const int x = cx - offset_x, y = cy - offset_y;
            if (x >= 0 && y >= 0 && x < grid.width() && y < grid.height())
              affected[Encode(x, y, yaw)] = true;
          }
    }
    for (std::size_t state = 0; state < affected.size(); ++state) if (affected[state])
      changed_states.push_back(static_cast<int>(state));
  }
  map_revision_ = costmap.revision();

  planning_internal::SearchOptions options;
  options.max_expansions = static_cast<std::size_t>(config_.lattice_max_expansions);
  options.max_planning_time_s = config_.lattice_max_planning_time;
  auto incremental = incremental_search_->Plan(source, changed_states, options);
  std::vector<int> states = incremental.path;
  double bound = 1.;
  planning_internal::SearchDiagnostics fallback_diagnostics;
  if (!incremental.found) {
    if (!cache_hit)
      obstacle_heuristic_.emplace(costmap, gx, gy, config_.robot_radius,
                                  config_.lattice_cost_penalty);
    const double bin_angle = 2. * M_PI / bins;
    const auto heuristic = [&](int state) {
      int x, y, yaw_bin; Decode(state, &x, &y, &yaw_bin);
      const double kinematic = std::max(
          std::hypot(gx - x, gy - y) * grid.resolution(),
          config_.lattice_rotation_cost * AngleDifference(yaw_bin * bin_angle, Yaw(goal)));
      return std::max(kinematic, obstacle_heuristic_->cost(x, y));
    };
    const auto fallback = planning_internal::AnytimeRepairingAStar(
        grid.width() * grid.height() * bins, source, target, heuristic,
        [this](int state, int, const auto& visit) { VisitSuccessors(state, visit); },
        config_.lattice_initial_heuristic_weight,
        config_.lattice_heuristic_weight_decrement, options);
    if (!fallback.found) throw std::runtime_error("no state lattice path");
    states = planning_internal::RestoreStatePath(fallback, source, target);
    fallback_diagnostics = fallback.diagnostics;
    bound = fallback.suboptimality_bound;
  }
  diagnostics_ = {
      incremental.diagnostics.expansions + fallback_diagnostics.expansions,
      incremental.diagnostics.generated + fallback_diagnostics.generated,
      incremental.diagnostics.elapsed_s + fallback_diagnostics.elapsed_s,
      incremental.found ? incremental.diagnostics.elapsed_s : fallback_diagnostics.first_solution_s,
      bound, cache_hit, incremental.reused, incremental.repaired_states};

  Path path{start};
  for (std::size_t index = 1; index < states.size(); ++index) {
    int x0, y0, yaw0, x1, y1, yaw1;
    Decode(states[index - 1], &x0, &y0, &yaw0);
    Decode(states[index], &x1, &y1, &yaw1);
    const auto* primitive = primitives_->Find(yaw0, x1 - x0, y1 - y0, yaw1);
    if (primitive == nullptr) throw std::runtime_error("missing lattice parent primitive");
    const auto [wx, wy] = grid.CellCenter(x0, y0);
    for (const auto& sample : primitive->samples)
      path.push_back(MakePose2d(wx + X(sample), wy + Y(sample), Yaw(sample)));
  }
  path.back() = goal;
  return planning_internal::ConstrainedPathSmoother(config_).Smooth(
      path, costmap, config_.robot_radius);
}

}  // namespace navigation2d
