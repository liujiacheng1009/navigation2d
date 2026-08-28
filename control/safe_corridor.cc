// DecompUtil adapter. Upstream: b0836c7228d19f0fa97282c584b55adf642279da, BSD-3-Clause.
#include "navigation2d/control/safe_corridor.h"

#include <algorithm>
#include <cmath>

#include <decomp_util/ellipsoid_decomp.h>

namespace navigation2d::control_internal {
namespace {

double FootprintSupport(const NavigationConfig& config, const Eigen::Vector2d& normal,
                        double yaw) {
  if (config.footprint.empty()) return config.robot_radius * normal.norm();
  const double cosine = std::cos(yaw), sine = std::sin(yaw);
  double support = 0.;
  for (const auto& point : config.footprint) {
    const Eigen::Vector2d rotated(cosine * point.x() - sine * point.y(),
                                  sine * point.x() + cosine * point.y());
    support = std::max(support, normal.dot(rotated));
  }
  return support;
}

}  // namespace

ConvexCorridor BuildSafeCorridor(const Path& path, const LayeredCostmap& costmap,
                                 const NavigationConfig& config) {
  if (path.size() < 2) return {};
  vec_Vec2f reference;
  reference.reserve(path.size());
  for (const auto& pose : path) {
    const Vec2f point(X(pose), Y(pose));
    if (reference.empty() || (point - reference.back()).norm() > 1e-4)
      reference.push_back(point);
  }
  if (reference.size() < 2) return {};

  vec_Vec2f obstacles;
  const auto& grid = costmap.grid();
  double min_x = reference.front().x(), max_x = min_x;
  double min_y = reference.front().y(), max_y = min_y;
  for (const auto& point : reference) {
    min_x = std::min(min_x, point.x()); max_x = std::max(max_x, point.x());
    min_y = std::min(min_y, point.y()); max_y = std::max(max_y, point.y());
  }
  const auto [first_x, first_y] = grid.ToCell(
      min_x - .5 * config.local_window_width, min_y - .5 * config.local_window_height);
  const auto [last_x, last_y] = grid.ToCell(
      max_x + .5 * config.local_window_width, max_y + .5 * config.local_window_height);
  for (int y = std::max(0, first_y); y <= std::min(grid.height() - 1, last_y); ++y)
  for (int x = std::max(0, first_x); x <= std::min(grid.width() - 1, last_x); ++x) {
    if (costmap.cost(x, y) < 253) continue;
    const auto [world_x, world_y] = grid.CellCenter(x, y);
    obstacles.emplace_back(world_x, world_y);
  }

  EllipsoidDecomp2D decomposition;
  decomposition.set_obs(obstacles);
  decomposition.set_local_bbox(Vec2f(config.local_window_width, config.local_window_height));
  decomposition.dilate(reference);
  const auto constraints = decomposition.get_constraints();
  ConvexCorridor corridor;
  corridor.reserve(constraints.size());
  for (std::size_t segment = 0; segment < constraints.size(); ++segment) {
    std::vector<Halfspace2d> halfspaces;
    const auto matrix = constraints[segment].A();
    const auto bounds = constraints[segment].b();
    const double yaw = Yaw(path[std::min(segment, path.size() - 1)]);
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
      Eigen::Vector2d normal = matrix.row(row).transpose();
      const double norm = normal.norm();
      if (norm < 1e-9) continue;
      normal /= norm;
      halfspaces.push_back({normal, bounds(row) / norm -
          FootprintSupport(config, normal, yaw)});
    }
    corridor.push_back(std::move(halfspaces));
  }
  return corridor;
}

}  // namespace navigation2d::control_internal
