#include "navigation2d/application/algorithm_registry.h"

#include <stdexcept>
#include <utility>

namespace navigation2d {

void AlgorithmRegistry::RegisterPlanner(std::string id, PlannerFactory factory,
                                        PlannerCapabilities capabilities) {
  if (id.empty() || !factory) {
    throw std::invalid_argument("invalid planner registration");
  }
  if (!planners_.emplace(std::move(id), PlannerEntry{std::move(factory), capabilities}).second) {
    throw std::invalid_argument("duplicate planner registration");
  }
}

void AlgorithmRegistry::RegisterController(std::string id, ControllerFactory factory,
                                           ControllerCapabilities capabilities) {
  if (id.empty() || !factory) {
    throw std::invalid_argument("invalid controller registration");
  }
  if (!controllers_.emplace(std::move(id), ControllerEntry{std::move(factory), capabilities})
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
  const auto capabilities = controllers_.at(config.controller).capabilities;
  if ((requirements.dynamic_predictions && !capabilities.dynamic_predictions) ||
      (requirements.guidance_candidates && !capabilities.guidance_candidates)) {
    throw std::invalid_argument("controller lacks required input capability: " + config.controller);
  }
}

std::unique_ptr<GlobalPlanner> AlgorithmRegistry::CreatePlanner(
    const NavigationConfig& config) const {
  const auto found = planners_.find(config.planner);
  if (found == planners_.end()) {
    throw std::runtime_error("unknown planner: " + config.planner);
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
