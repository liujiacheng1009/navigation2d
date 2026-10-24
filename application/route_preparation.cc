#include "navigation2d/application/route_preparation.h"

#include <stdexcept>
#include <utility>

#include "navigation2d/planning/astar_planner.h"
#include "navigation2d/planning/path_geometry.h"
#include "navigation2d/planning/path_smoother.h"
#include "navigation2d/planning/route_identity.h"

namespace navigation2d {
namespace {
using planning_internal::DensifyRoute;
using planning_internal::FirstSegmentSignature;
using planning_internal::PathFootprintValid;

Path PlanWithFallback(GlobalPlanner& planner, const LayeredCostmap& map, double radius,
                      PlannerCapabilities traits, const RouteRequest& request) {
  try {
    return DensifyRoute(planner.Plan(map, request.start, request.goal), .08);
  } catch (const std::runtime_error&) {
    // Preserve the lattice's existing circular-grid fallback when polygon
    // admission rejects a route accepted by the navigation execution contract.
    if (!traits.grid_fallback) {
      throw;
    }
    return DensifyRoute(AStarPlanner(radius).Plan(map, request.start, request.goal), .08);
  }
}

Path ValidateCorners(Path path, const LayeredCostmap& map, double radius,
                     PlannerCapabilities traits, const RouteRequest& request) {
  if (!PathFootprintValid(path, map, radius)) {
    // Visibility chords and densified lattice segments may cut a corner.
    if (traits.corner_grid_retry) {
      path = DensifyRoute(AStarPlanner(radius).Plan(map, request.start, request.goal), .08);
    }
    if (!PathFootprintValid(path, map, radius)) {
      throw std::runtime_error("planned path failed dense footprint validation");
    }
  }
  return path;
}

PreparedRoute AvoidCooledRoute(PreparedRoute route, const LayeredCostmap& map,
                               const NavigationConfig& config, const RouteRequest& request) {
  if (request.banned_signatures.count(route.signature) == 0) {
    return route;
  }
  // One A* retry can change the first segment. Replaying an already cooled
  // signature is an execution failure, not a newly admitted route.
  if (config.planner == "astar") {
    throw std::runtime_error("failed first-segment replay");
  }
  auto alternate =
      DensifyRoute(AStarPlanner(config.robot_radius).Plan(map, request.start, request.goal), .08);
  if (!PathFootprintValid(alternate, map, config.robot_radius)) {
    throw std::runtime_error("failed first-segment replay");
  }
  const auto signature = FirstSegmentSignature(alternate, request.start, map.grid().resolution());
  if (request.banned_signatures.count(signature) != 0) {
    throw std::runtime_error("failed first-segment replay");
  }
  return {std::move(alternate), signature};
}

Path SmoothRoute(Path path, const LayeredCostmap& map, const NavigationConfig& config,
                 PlannerCapabilities traits) {
  // Lattice already smooths in Plan. Other planners retain the original route
  // if smoothing or subsequent densification invalidates the footprint.
  if (traits.owns_smoothing || !config.smoother_enabled) {
    return path;
  }
  Path smoothed =
      planning_internal::ConstrainedPathSmoother(config).Smooth(path, map, config.robot_radius);
  if (PathFootprintValid(smoothed, map, config.robot_radius)) {
    Path dense = DensifyRoute(smoothed, .08);
    if (PathFootprintValid(dense, map, config.robot_radius)) {
      path = std::move(dense);
    }
  }
  return path;
}
}  // namespace

PreparedRoute PrepareRoute(GlobalPlanner& planner, const LayeredCostmap& costmap,
                           const NavigationConfig& config, PlannerCapabilities traits,
                           const RouteRequest& request) {
  auto path = PlanWithFallback(planner, costmap, config.robot_radius, traits, request);
  path = ValidateCorners(std::move(path), costmap, config.robot_radius, traits, request);
  const auto signature = FirstSegmentSignature(path, request.start, costmap.grid().resolution());
  auto route = AvoidCooledRoute({std::move(path), signature}, costmap, config, request);
  // The legacy identity is measured before smoothing; do not recompute it.
  route.path = SmoothRoute(std::move(route.path), costmap, config, traits);
  return route;
}
}  // namespace navigation2d
