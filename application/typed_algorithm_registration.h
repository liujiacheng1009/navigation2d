#pragma once

#include <string>
#include <utility>

#include "navigation2d/application/algorithm_registry.h"

namespace navigation2d {
// Parser(common, parameters) returns an algorithm-owned typed configuration
// and rejects unknown keys/range errors. Validation runs without constructing
// a backend. Factory receives only that configuration, never a YAML node.
// Callable copies must be safe to share after registry assembly.
template <class Parser, class Factory>
void RegisterTypedPlanner(AlgorithmRegistry& registry, std::string id, Parser parser,
                          Factory factory, PlannerCapabilities capabilities = {}) {
  const auto name = id;
  registry.RegisterPlanner(
      std::move(id),
      [name, parser, factory](const NavigationConfig& common) {
        return factory(parser(common, ParametersFor(common.global_planners, name)));
      },
      capabilities,
      [parser](const NavigationConfig& common, const AlgorithmParameters& parameters) {
        static_cast<void>(parser(common, parameters));
      });
}

template <class Parser, class Factory>
void RegisterTypedController(AlgorithmRegistry& registry, std::string id, Parser parser,
                             Factory factory, ControllerCapabilities capabilities = {}) {
  const auto name = id;
  registry.RegisterController(
      std::move(id),
      [name, parser, factory](const NavigationConfig& common) {
        return factory(parser(common, ParametersFor(common.local_controllers, name)));
      },
      capabilities,
      [parser](const NavigationConfig& common, const AlgorithmParameters& parameters) {
        static_cast<void>(parser(common, parameters));
      });
}
}  // namespace navigation2d
