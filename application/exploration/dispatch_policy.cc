#include "navigation2d/application/exploration/dispatch_policy.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/application/exploration/navigation_session.h"
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
DispatchVerdict DispatchPolicy::RejectUnreachable(const ExplorationGoal& goal,
                                                  DispatchPurpose purpose, bool latest_map_retry,
                                                  const InitialPlanResult& probe,
                                                  DispatchActions& actions) {
  const auto& state = probe.state;
  actions.RejectPreparation(
      {.kind = DispatchRejection::kNoRoute,
       .reason = probe.failure_reason(),
       .start_occupied = probe.failure == InitialPlanFailure::kStartOccupied});
  if (probe.failure != InitialPlanFailure::kNoRoute &&
      probe.failure != InitialPlanFailure::kStartOccupied &&
      probe.failure != InitialPlanFailure::kGoalOccupied) { return DispatchVerdict::kRejected; }
  const auto reason = state.planning_failure_reason.empty()
                          ? std::string_view("no route")
                          : std::string_view(state.planning_failure_reason);
  if (!purpose.returning && !purpose.retreat && PinchWorthy(reason, purpose.entry) &&
      Retry(RetryKind::kPinch, [&] {
        actions.ReportDispatchRetry({DispatchRetryNotice::Kind::kNoRoutePinch, purpose.entry});
        return actions.RetryNavigation(
            goal, {.returning = purpose.returning, .latest_map = latest_map_retry});
      })) {
    return DispatchVerdict::kAcceptedRetry;
  }
  return DispatchVerdict::kRejected;
}
DispatchVerdict DispatchPolicy::RejectMappedWall(const ExplorationGoal& goal,
                                                 DispatchPurpose purpose, std::uint64_t signature,
                                                 DispatchActions& actions) {
  actions.RejectPreparation(
      {.kind = DispatchRejection::kMappedWall, .reason = "path intersects mapped obstacle"});
  // Entry candidates share their hallway prefix. Cool that signature only
  // after the retry fails; open/return candidates cool before retry as well.
  if (Retry(RetryKind::kMappedWall, [&] {
        if (!purpose.entry && signature != 0) {
          actions.CoolRouteSignature(signature);
        }
        return actions.RetryNavigation(goal, {.returning = purpose.returning, .latest_map = true});
      })) {
    return DispatchVerdict::kAcceptedRetry;
  }
  if (signature != 0) {
    actions.CoolRouteSignature(signature);
  }
  if (!purpose.entry) {
    actions.NoteMappedObstacleReject(goal);
  }
  return DispatchVerdict::kRejected;
}
DispatchVerdict DispatchPolicy::Evaluate(const ExplorationGoal& goal, DispatchPurpose purpose,
                                         bool latest_map_retry, const InitialPlanResult& probe,
                                         DispatchActions& actions) {
  const auto& state = probe.state;
  const auto signature = state.path_signature;
  if (!probe.has_route()) {
    return RejectUnreachable(goal, purpose, latest_map_retry, probe, actions);
  }
  if (actions.PreparedRouteCutsWall()) {
    return RejectMappedWall(goal, purpose, signature, actions);
  }
  const double margin = state.global_planning.path_min_clearance_m;
  actions.ObservePreparedRoute(signature, margin);
  const auto acceptance = CheckClearance(purpose, margin);
  if (acceptance == RouteAcceptance::kOpenScrape) {
    if (signature != 0) {
      actions.CoolRouteSignature(signature);
    }
    actions.RejectPreparation({.kind = DispatchRejection::kOpenScrape,
                               .reason = "wall-scrape path in open clearance",
                               .margin_m = margin});
    if (Retry(RetryKind::kOpenScrape, [&] {
          return actions.RetryNavigation(
              goal, {.returning = purpose.returning, .latest_map = latest_map_retry});
        })) {
      return DispatchVerdict::kAcceptedRetry;
    }
    // A whole-map pinch retry is allowed here only for a passable mouth.
    if (!active(RetryKind::kPinch) && actions.PassableMouthGoal(goal) &&
        Retry(RetryKind::kPinch, [&] {
          actions.ReportDispatchRetry({DispatchRetryNotice::Kind::kScrapePinch, purpose.entry});
          return actions.RetryNavigation(
              goal, {.returning = purpose.returning, .latest_map = latest_map_retry});
        })) {
      return DispatchVerdict::kAcceptedRetry;
    }
    return DispatchVerdict::kRejected;
  }
  if (acceptance == RouteAcceptance::kPinchScrape) {
    if (signature != 0) {
      actions.CoolRouteSignature(signature);
    }
    actions.RejectPreparation({.kind = DispatchRejection::kPinchScrape,
                               .reason = "pinch path still wall-scrape",
                               .margin_m = margin});
    return DispatchVerdict::kRejected;
  }
  return DispatchVerdict::kAcceptPrepared;
}
}  // namespace navigation2d::exploration
