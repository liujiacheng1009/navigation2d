#pragma once
#include <string>
namespace navigation2d {
enum class RouteHeadingPolicy { kLegacyForwardProjection, kPreservePlannerHeading };
struct NavigationExecutionOptions {
  // Scoped to this navigation instance; poses/map/path must share this frame.
  std::string frame_id = "navigation_map";
  RouteHeadingPolicy heading_policy = RouteHeadingPolicy::kLegacyForwardProjection;
};
}  // namespace navigation2d
