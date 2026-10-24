#pragma once

#include <cstdint>
#include <unordered_set>

#include "navigation2d/application/algorithm_registry.h"

namespace navigation2d {
struct RouteRequest {
  const Pose2d& start;
  const Pose2d& goal;
  const std::unordered_set<std::uint64_t>& banned_signatures;
};
struct PreparedRoute {
  Path path;
  std::uint64_t signature = 0;
};
// Synchronous, borrowed inputs in one local map frame. Preserves planning,
// fallback, signature cooling, smoothing and dense validation order. Does not
// commit navigation progress, rebuild a controller or publish commands.
// Existing runtime_error diagnostics propagate to the navigation boundary.
PreparedRoute PrepareRoute(GlobalPlanner& planner, const LayeredCostmap& costmap,
                           const NavigationConfig& config, PlannerCapabilities traits,
                           const RouteRequest& request);
}  // namespace navigation2d
