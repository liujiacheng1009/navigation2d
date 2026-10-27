#include "navigation2d/application/algorithm_registry.h"

#include <stdexcept>
#include <utility>

namespace navigation2d {

void AlgorithmRegistry::RegisterPlanner(std::string id, PlannerFactory factory,
                                        PlannerCapabilities capabilities,
                                        ParameterValidator validator) {
  if (id.empty() || !factory) {
    throw std::invalid_argument("invalid planner registration");
  }
  if (!planners_
           .emplace(std::move(id),
                    PlannerEntry{std::move(factory), capabilities, std::move(validator)})
           .second) {
    throw std::invalid_argument("duplicate planner registration");
  }
}

void AlgorithmRegistry::RegisterController(std::string id, ControllerFactory factory,
                                           ControllerCapabilities capabilities,
                                           ParameterValidator validator) {
  if (id.empty() || !factory) {
    throw std::invalid_argument("invalid controller registration");
  }
  if (!controllers_
           .emplace(std::move(id),
                    ControllerEntry{std::move(factory), capabilities, std::move(validator)})
           .second) {
    throw std::invalid_argument("duplicate controller registration");
  }
}

void AlgorithmRegistry::Validate(const NavigationConfig& config,
                                 AlgorithmRequirements requirements) const {
  if (!planners_.contains(config.planner)) {
    throw std::runtime_error("unknown planner: " + config.planner);
  }
  if (!controllers_.contains(config.controller)) {
    throw std::runtime_error("unknown controller: " + config.controller);
  }
  const auto validate = [&](const auto& sets, const auto& entries, const std::string& path) {
    for (const auto& [id, parameters] : sets) {
      const auto found = entries.find(id);
      if (found == entries.end()) {
        throw std::invalid_argument("unknown algorithm namespace: " + path + "." + id);
      }
      if (found->second.validator) {
        found->second.validator(config, parameters);
      } else if (!parameters.empty()) {
        throw std::invalid_argument("algorithm has no parameter descriptor: " + path + "." + id);
      }
    }
  };
  validate(config.global_planners, planners_, "global_planners");
  validate(config.local_controllers, controllers_, "local_controllers");
  if (!config.global_planners.contains(config.planner) && planners_.at(config.planner).validator) {
    planners_.at(config.planner)
        .validator(config, ParametersFor(config.global_planners, config.planner));
  }
  if (!config.local_controllers.contains(config.controller) &&
      controllers_.at(config.controller).validator) {
    controllers_.at(config.controller)
        .validator(config, ParametersFor(config.local_controllers, config.controller));
  }
  const auto capabilities = controllers_.at(config.controller).capabilities;
  if ((requirements.dynamic_predictions && !capabilities.dynamic_predictions) ||
      (requirements.guidance_candidates && !capabilities.guidance_candidates) ||
      (requirements.reverse_paths && (!planners_.at(config.planner).capabilities.reverse_paths ||
                                      !capabilities.reverse_tracking)) ||
      (requirements.in_place_rotation && !capabilities.in_place_rotation)) {
    throw std::invalid_argument("algorithm combination lacks required capability: " +
                                config.controller);
  }
}

std::unique_ptr<GlobalPlanner> AlgorithmRegistry::CreatePlanner(
    const NavigationConfig& config) const {
  const auto found = planners_.find(config.planner);
  if (found == planners_.end()) {
    throw std::runtime_error("unknown planner: " + config.planner);
  }
  const auto& parameters = ParametersFor(config.global_planners, config.planner);
  if (found->second.validator) {
    found->second.validator(config, parameters);
  } else if (!parameters.empty()) {
    throw std::invalid_argument("algorithm has no parameter descriptor: " + config.planner);
  }
  auto result = found->second.factory(config);
  if (!result) {
    throw std::runtime_error("planner factory returned null: " + config.planner);
  }
  return result;
}

std::unique_ptr<LocalController> AlgorithmRegistry::CreateController(
    const NavigationConfig& config) const {
  const auto found = controllers_.find(config.controller);
  if (found == controllers_.end()) {
    throw std::runtime_error("unknown controller: " + config.controller);
  }
  const auto& parameters = ParametersFor(config.local_controllers, config.controller);
  if (found->second.validator) {
    found->second.validator(config, parameters);
  } else if (!parameters.empty()) {
    throw std::invalid_argument("algorithm has no parameter descriptor: " + config.controller);
  }
  auto result = found->second.factory(config);
  if (!result) {
    throw std::runtime_error("controller factory returned null: " + config.controller);
  }
  return result;
}

PlannerCapabilities AlgorithmRegistry::PlannerTraits(const std::string& id) const {
  const auto found = planners_.find(id);
  if (found == planners_.end()) {
    throw std::runtime_error("unknown planner: " + id);
  }
  return found->second.capabilities;
}

}  // namespace navigation2d
