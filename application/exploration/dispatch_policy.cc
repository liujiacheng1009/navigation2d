#include "navigation2d/application/exploration/dispatch_policy.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
bool DispatchPolicy::PinchWorthy(std::string_view failure, bool entry) {
  return failure.find("goal occupied") == std::string_view::npos &&
         (failure.find("no route") != std::string_view::npos ||
          failure.find("no path") != std::string_view::npos ||
          (entry && failure.find("first-segment") != std::string_view::npos));
}
RouteAcceptance DispatchPolicy::CheckClearance(DispatchPurpose purpose, double margin_m) {
  if (purpose.entry || purpose.returning || purpose.retreat || !std::isfinite(margin_m)) {
    return RouteAcceptance::kAccept;
  }
  if (!purpose.pinch && !PathClearsOpen(margin_m)) {
    return RouteAcceptance::kOpenScrape;
  }
  if (purpose.pinch && !PathClearsPinch(margin_m)) {
    return RouteAcceptance::kPinchScrape;
  }
  return RouteAcceptance::kAccept;
}
double DispatchPolicy::GoalTimeout(bool returning, double length_m) {
  return returning ? std::max(55., length_m / .10 + 20.)
                   : std::min(75., std::max(40., length_m / .12 + 15.));
}
}  // namespace navigation2d::exploration
