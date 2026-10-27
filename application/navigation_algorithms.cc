#include "navigation2d/application/navigation_algorithms.h"

#include <cmath>
#include <exception>
#include <utility>

namespace navigation2d {
NavigationAlgorithms::NavigationAlgorithms(const NavigationConfig& config,
                                           std::shared_ptr<const AlgorithmRegistry> registry)
    : config_(config),
      registry_(std::move(registry)),
      planner_(registry_->CreatePlanner(config_)),
      controller_(registry_->CreateController(config_)),
      planner_traits_(registry_->PlannerTraits(config_.planner)),
      controller_traits_(registry_->ControllerTraits(config_.controller)) {}

PreparedRoute NavigationAlgorithms::Prepare(const LayeredCostmap& map,
                                            const RouteRequest& request) {
  auto route = PrepareRoute(*planner_, map, config_, planner_traits_, request);
  if (route.metadata.headings != PathHeadingSemantics::kBodyHeading) {
    return route;
  }
  if (!controller_traits_.body_heading_paths) {
    throw RoutePlanningError(GlobalPlanStatus::kInvalidInput,
                             "controller cannot track body-heading paths");
  }
  for (std::size_t i = 1; i < route.path.size(); ++i) {
    const auto delta = route.path[i].translation() - route.path[i - 1].translation();
    const double yaw = Yaw(route.path[i - 1]);
    const double forward = delta.x() * std::cos(yaw) + delta.y() * std::sin(yaw);
    if (forward < -1e-8 &&
        (!planner_traits_.reverse_paths || !controller_traits_.reverse_tracking)) {
      throw RoutePlanningError(GlobalPlanStatus::kInvalidInput,
                               "algorithm combination cannot track reverse paths");
    }
    if (delta.norm() <= 1e-9 &&
        std::abs(std::atan2(std::sin(Yaw(route.path[i]) - yaw),
                            std::cos(Yaw(route.path[i]) - yaw))) > 1e-8 &&
        !controller_traits_.in_place_rotation) {
      throw RoutePlanningError(GlobalPlanStatus::kInvalidInput,
                               "controller cannot execute a path rotation");
    }
  }
  return route;
}
void NavigationAlgorithms::ResetController() {
  try {
    controller_ = registry_->CreateController(config_);
  } catch (const std::exception& error) {
    throw RoutePlanningError(GlobalPlanStatus::kBackendError, error.what());
  } catch (...) {
    throw RoutePlanningError(GlobalPlanStatus::kBackendError, "non-standard backend exception");
  }
}
GlobalPlanningDiagnostics NavigationAlgorithms::PlanningDiagnostics() const {
  try {
    return planner_->Diagnostics();
  } catch (const std::exception& error) {
    throw RoutePlanningError(GlobalPlanStatus::kBackendError, error.what());
  } catch (...) {
    throw RoutePlanningError(GlobalPlanStatus::kBackendError, "non-standard backend exception");
  }
}
LocalControlResult NavigationAlgorithms::Compute(const LocalControlRequest& request) const {
  auto result = controller_->ComputeWithContext(request);
  if (result.has_command() &&
      (result.command.linear > config_.desired_linear_velocity + 1e-9 ||
       result.command.linear < -config_.max_reverse_velocity - 1e-9 ||
       std::abs(result.command.angular) > config_.max_angular_velocity + 1e-9)) {
    result.status = LocalControlStatus::kInvalidOutput;
    result.command = {};
    result.reason = "controller exceeded common motion limits";
    if (result.diagnostics) {
      result.diagnostics->status = ControllerSolveStatus::kUnsafe;
    }
  }
  return result;
}
void NavigationAlgorithms::SetGuidance(std::vector<GuidanceCandidate> candidates) {
  controller_->SetGuidanceCandidates(std::move(candidates));
}
}  // namespace navigation2d
