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

GlobalPlanResult RequirePlan(GlobalPlanner& planner, const LayeredCostmap& map,
                             const RouteRequest& request) {
  auto result =
      planner.PlanWithContext({map, request.start, request.goal, request.frame_id, request.budget});
  if (!result.succeeded()) {
    throw RoutePlanningError(result.status,
                             result.reason.empty() ? "global planning failed" : result.reason);
  }
  if (request.heading_policy == RouteHeadingPolicy::kPreservePlannerHeading &&
      result.metadata.headings == PathHeadingSemantics::kUnspecified) {
    throw RoutePlanningError(GlobalPlanStatus::kInvalidOutput, "planner omitted heading semantics");
  }
  if (request.heading_policy == RouteHeadingPolicy::kLegacyForwardProjection &&
      result.metadata.headings == PathHeadingSemantics::kBodyHeading &&
      !result.metadata.legacy_forward_projection_allowed) {
    throw RoutePlanningError(GlobalPlanStatus::kInvalidInput,
                             "body-heading path requires preservation policy");
  }
  const bool body = request.heading_policy == RouteHeadingPolicy::kPreservePlannerHeading &&
                    result.metadata.headings == PathHeadingSemantics::kBodyHeading;
  result.path = body ? planning_internal::DensifyBodyHeadingRoute(result.path, .08)
                     : DensifyRoute(result.path, .08);
  if (!body) {
    result.metadata.headings = PathHeadingSemantics::kForwardTangent;
  }
  return result;
}

GlobalPlanResult GridPlan(const LayeredCostmap& map, double radius, const RouteRequest& request) {
  AStarPlanner fallback(radius);
  return RequirePlan(fallback, map, request);
}

GlobalPlanResult PlanWithFallback(GlobalPlanner& planner, const LayeredCostmap& map, double radius,
                                  PlannerCapabilities traits, const RouteRequest& request) {
  try {
    return RequirePlan(planner, map, request);
  } catch (const RoutePlanningError& error) {
    // Cancellation, invalid data and late output are not no-route retries.
    if (!traits.grid_fallback || error.status() == GlobalPlanStatus::kCancelled ||
        error.status() == GlobalPlanStatus::kBudgetExceeded ||
        error.status() == GlobalPlanStatus::kInvalidInput ||
        error.status() == GlobalPlanStatus::kInvalidOutput) {
      throw;
    }
    return GridPlan(map, radius, request);
  }
}

GlobalPlanResult ValidateCorners(GlobalPlanResult result, const LayeredCostmap& map, double radius,
                                 PlannerCapabilities traits, const RouteRequest& request) {
  if (!PathFootprintValid(result.path, map, radius)) {
    if (traits.corner_grid_retry) {
      result = GridPlan(map, radius, request);
    }
    if (!PathFootprintValid(result.path, map, radius)) {
      throw RoutePlanningError(GlobalPlanStatus::kNoPath,
                               "planned path failed dense footprint validation",
                               RouteRejection::kFootprintInvalid);
    }
  }
  return result;
}

PreparedRoute AvoidCooledRoute(PreparedRoute route, const LayeredCostmap& map,
                               const NavigationConfig& config, const RouteRequest& request) {
  if (request.banned_signatures.count(route.signature) == 0) {
    return route;
  }
  // One A* retry can change the first segment. Replaying an already cooled
  // signature is an execution failure, not a newly admitted route.
  if (config.planner == "astar") {
    throw RoutePlanningError(GlobalPlanStatus::kNoPath, "failed first-segment replay",
                             RouteRejection::kSignatureExcluded);
  }
  auto planned = GridPlan(map, config.robot_radius, request);
  auto alternate = std::move(planned.path);
  if (!PathFootprintValid(alternate, map, config.robot_radius)) {
    throw RoutePlanningError(GlobalPlanStatus::kNoPath, "failed first-segment replay",
                             RouteRejection::kSignatureExcluded);
  }
  const auto signature = FirstSegmentSignature(alternate, request.start, map.grid().resolution());
  if (request.banned_signatures.count(signature) != 0) {
    throw RoutePlanningError(GlobalPlanStatus::kNoPath, "failed first-segment replay",
                             RouteRejection::kSignatureExcluded);
  }
  return {std::move(alternate), signature, std::move(planned.metadata)};
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
  auto planned = PlanWithFallback(planner, costmap, config.robot_radius, traits, request);
  planned = ValidateCorners(std::move(planned), costmap, config.robot_radius, traits, request);
  const auto signature =
      FirstSegmentSignature(planned.path, request.start, costmap.grid().resolution());
  auto route = AvoidCooledRoute({std::move(planned.path), signature, std::move(planned.metadata)},
                                costmap, config, request);
  // Legacy identity is measured before smoothing. Body-heading paths own their
  // yaw/rotation/cusp contract and cannot pass through a tangent-only smoother.
  if (route.metadata.headings != PathHeadingSemantics::kBodyHeading) {
    route.path = SmoothRoute(std::move(route.path), costmap, config, traits);
  }
  return route;
}
}  // namespace navigation2d
