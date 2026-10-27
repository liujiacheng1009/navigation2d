#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "navigation2d/application/algorithm_capabilities.h"
#include "navigation2d/configuration/navigation_config.h"
#include "navigation2d/control/local_controller.h"
#include "navigation2d/planning/global_planner.h"

namespace navigation2d {

// Build during application assembly, then share as const. No lookup occurs
// inside a controller Compute call. Factories must return independently owned
// instances; existing reset/replan semantics remain owned by NavigationSystem.
class AlgorithmRegistry {
 public:
  using ParameterValidator =
      std::function<void(const NavigationConfig&, const AlgorithmParameters&)>;
  using PlannerFactory = std::function<std::unique_ptr<GlobalPlanner>(const NavigationConfig&)>;
  using ControllerFactory =
      std::function<std::unique_ptr<LocalController>(const NavigationConfig&)>;

  void RegisterPlanner(std::string id, PlannerFactory factory,
                       PlannerCapabilities capabilities = {}, ParameterValidator validator = {});
  void RegisterController(std::string id, ControllerFactory factory,
                          ControllerCapabilities capabilities = {},
                          ParameterValidator validator = {});
  [[nodiscard]] std::unique_ptr<GlobalPlanner> CreatePlanner(const NavigationConfig& config) const;
  [[nodiscard]] std::unique_ptr<LocalController> CreateController(
      const NavigationConfig& config) const;
  void Validate(const NavigationConfig& config, AlgorithmRequirements requirements = {}) const;
  PlannerCapabilities PlannerTraits(const std::string& id) const;
  ControllerCapabilities ControllerTraits(const std::string& id) const;
  static std::shared_ptr<const AlgorithmRegistry> Builtins();

 private:
  struct PlannerEntry {
    PlannerFactory factory;
    PlannerCapabilities capabilities;
    ParameterValidator validator;
  };
  struct ControllerEntry {
    ControllerFactory factory;
    ControllerCapabilities capabilities;
    ParameterValidator validator;
  };
  std::unordered_map<std::string, PlannerEntry> planners_;
  std::unordered_map<std::string, ControllerEntry> controllers_;
};

}  // namespace navigation2d
