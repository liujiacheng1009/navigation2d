#pragma once
#include <array>
#include <string_view>

#include "navigation2d/application/algorithm_capabilities.h"
namespace navigation2d::builtin_algorithms {
// Names and capabilities are metadata: configuration readers do not construct
// a registry or require concrete planner/controller implementations.
enum class PlannerId { kDijkstra, kAStar, kThetaStar, kStateLattice };
enum class ControllerId { kRpp, kDwa, kMppi, kMpc };
struct PlannerSpec {
  std::string_view name;
  PlannerId kind;
  PlannerCapabilities capabilities;
};
struct ControllerSpec {
  std::string_view name;
  ControllerId kind;
  ControllerCapabilities capabilities;
};
inline constexpr std::array<PlannerSpec, 4> kPlanners{{
    {"dijkstra", PlannerId::kDijkstra, {}},
    {"astar", PlannerId::kAStar, {}},
    {"theta_star", PlannerId::kThetaStar, {false, false, true}},
    {"state_lattice", PlannerId::kStateLattice, {true, true, true}},
}};
inline constexpr std::array<ControllerSpec, 4> kControllers{{
    {"rpp", ControllerId::kRpp, {}},
    {"dwa", ControllerId::kDwa, {}},
    {"mppi", ControllerId::kMppi, {}},
    {"mpc", ControllerId::kMpc, {true, true}},
}};
constexpr bool HasPlanner(std::string_view name) noexcept {
  for (const auto& spec : kPlanners) {
    if (spec.name == name) {
      return true;
    }
  }
  return false;
}
constexpr bool HasController(std::string_view name) noexcept {
  for (const auto& spec : kControllers) {
    if (spec.name == name) {
      return true;
    }
  }
  return false;
}
}  // namespace navigation2d::builtin_algorithms
