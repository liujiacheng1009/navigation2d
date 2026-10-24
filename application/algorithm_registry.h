#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "navigation2d/configuration/navigation_config.h"
#include "navigation2d/control/local_controller.h"
#include "navigation2d/planning/global_planner.h"

namespace navigation2d {

// Build during application assembly, then share as const. No lookup occurs
// inside a controller Compute call. Factories must return independently owned
// instances; existing reset/replan semantics remain owned by NavigationSystem.
struct PlannerCapabilities {
  bool owns_smoothing = false;
  bool grid_fallback = false;
  bool corner_grid_retry = false;
};
struct ControllerCapabilities {
  bool dynamic_predictions = false;
  bool guidance_candidates = false;
};
struct AlgorithmRequirements {
  bool dynamic_predictions = false;
  bool guidance_candidates = false;
};
class AlgorithmRegistry {
 public:
  using PlannerFactory = std::function<std::unique_ptr<GlobalPlanner>(const NavigationConfig&)>;
  using ControllerFactory =
      std::function<std::unique_ptr<LocalController>(const NavigationConfig&)>;

  void RegisterPlanner(std::string id, PlannerFactory factory,
                       PlannerCapabilities capabilities = {});
  void RegisterController(std::string id, ControllerFactory factory,
                          ControllerCapabilities capabilities = {});
  [[nodiscard]] std::unique_ptr<GlobalPlanner> CreatePlanner(const NavigationConfig& config) const;
  [[nodiscard]] std::unique_ptr<LocalController> CreateController(
      const NavigationConfig& config) const;
  void Validate(const NavigationConfig& config, AlgorithmRequirements requirements = {}) const;
  PlannerCapabilities PlannerTraits(const std::string& id) const;
  static std::shared_ptr<const AlgorithmRegistry> Builtins();

 private:
  struct PlannerEntry {
    PlannerFactory factory;
    PlannerCapabilities capabilities;
  };
  struct ControllerEntry {
    ControllerFactory factory;
    ControllerCapabilities capabilities;
  };
  std::unordered_map<std::string, PlannerEntry> planners_;
  std::unordered_map<std::string, ControllerEntry> controllers_;
};

}  // namespace navigation2d
