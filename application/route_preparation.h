#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include "navigation2d/application/algorithm_registry.h"
#include "navigation2d/application/navigation_execution_options.h"

namespace navigation2d {
struct RouteRequest {
  const Pose2d& start;
  const Pose2d& goal;
  const std::unordered_set<std::uint64_t>& banned_signatures;
  std::string_view frame_id = "navigation_map";
  AlgorithmBudget budget{};
  RouteHeadingPolicy heading_policy = RouteHeadingPolicy::kLegacyForwardProjection;
};
struct PreparedRoute {
  Path path;
  std::uint64_t signature = 0;
  PlannedPathMetadata metadata{};
};
class RoutePlanningError : public std::runtime_error {
 public:
  RoutePlanningError(GlobalPlanStatus status, std::string reason)
      : std::runtime_error(std::move(reason)), status_(status) {}
  GlobalPlanStatus status() const { return status_; }

 private:
  GlobalPlanStatus status_;
};
// Synchronous, borrowed inputs in one local map frame. Preserves planning,
// fallback, signature cooling, smoothing and dense validation order. Does not
// commit navigation progress, rebuild a controller or publish commands.
// Existing runtime_error diagnostics propagate to the navigation boundary.
PreparedRoute PrepareRoute(GlobalPlanner& planner, const LayeredCostmap& costmap,
                           const NavigationConfig& config, PlannerCapabilities traits,
                           const RouteRequest& request);
}  // namespace navigation2d
