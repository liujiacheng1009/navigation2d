#pragma once

#include <memory>
#include <vector>

#include "navigation2d/application/algorithm_registry.h"
#include "navigation2d/application/route_preparation.h"

namespace navigation2d {
// Owns the selected backend instances and their declared capabilities. Single
// writer. The borrowed immutable configuration must outlive this object.
class NavigationAlgorithms {
 public:
  NavigationAlgorithms(const NavigationConfig& config,
                       std::shared_ptr<const AlgorithmRegistry> registry);
  NavigationAlgorithms(const NavigationConfig&&, std::shared_ptr<const AlgorithmRegistry>) = delete;
  PreparedRoute Prepare(const LayeredCostmap& map, const RouteRequest& request);
  void ResetController();
  GlobalPlanningDiagnostics PlanningDiagnostics() const;
  LocalControlResult Compute(const LocalControlRequest& request) const;
  void SetGuidance(std::vector<GuidanceCandidate> candidates);
  PlannerCapabilities planner_traits() const { return planner_traits_; }

 private:
  const NavigationConfig& config_;
  std::shared_ptr<const AlgorithmRegistry> registry_;
  std::unique_ptr<GlobalPlanner> planner_;
  std::unique_ptr<LocalController> controller_;
  PlannerCapabilities planner_traits_;
  ControllerCapabilities controller_traits_;
};
}  // namespace navigation2d
