#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "navigation2d/application/algorithm_registry.h"
#include "navigation2d/application/builtin_algorithm_catalog.h"
#include "navigation2d/configuration/builtin_parameters.h"
#include "navigation2d/control/dwa_controller.h"
#include "navigation2d/control/mpc_controller.h"
#include "navigation2d/control/mppi_controller.h"
#include "navigation2d/control/regulated_pure_pursuit.h"
#include "navigation2d/planning/astar_planner.h"
#include "navigation2d/planning/dijkstra_planner.h"
#include "navigation2d/planning/state_lattice_planner.h"
#include "navigation2d/planning/theta_star_planner.h"
namespace navigation2d {
namespace {
std::unique_ptr<GlobalPlanner> Planner(builtin_algorithms::PlannerId kind,
                                       const NavigationConfig& c) {
  using builtin_algorithms::PlannerId;
  switch (kind) {
    case PlannerId::kDijkstra:
      return std::make_unique<DijkstraPlanner>(c.robot_radius);
    case PlannerId::kAStar:
      return std::make_unique<AStarPlanner>(c.robot_radius);
    case PlannerId::kThetaStar:
      return std::make_unique<ThetaStarPlanner>(c.robot_radius);
    case PlannerId::kStateLattice:
      return std::make_unique<StateLatticePlanner>(c);
  }
  throw std::logic_error("invalid builtin planner kind");
}
std::unique_ptr<LocalController> Controller(builtin_algorithms::ControllerId kind,
                                            const NavigationConfig& c) {
  using builtin_algorithms::ControllerId;
  switch (kind) {
    case ControllerId::kRpp:
      return std::make_unique<RegulatedPurePursuit>(c);
    case ControllerId::kDwa:
      return std::make_unique<DwaController>(c);
    case ControllerId::kMppi:
      return std::make_unique<MppiController>(c);
    case ControllerId::kMpc:
      return std::make_unique<MpcController>(c);
  }
  throw std::logic_error("invalid builtin controller kind");
}
}  // namespace
std::shared_ptr<const AlgorithmRegistry> AlgorithmRegistry::Builtins() {
  static const auto registry = [] {
    auto result = std::make_shared<AlgorithmRegistry>();
    for (const auto& spec : builtin_algorithms::kPlanners) {
      result->RegisterPlanner(
          std::string(spec.name),
          [kind = spec.kind, id = std::string(spec.name)](const auto& c) {
            if (ParametersFor(c.global_planners, id).empty()) {
              return Planner(kind, c);
            }
            return Planner(kind, ResolveBuiltinParameters(c, id, true));
          },
          spec.capabilities,
          [id = std::string(spec.name)](const auto& c, const auto& parameters) {
            if (!parameters.empty()) {
              static_cast<void>(ResolveBuiltinParameters(c, id, true));
            }
          });
    }
    for (const auto& spec : builtin_algorithms::kControllers) {
      result->RegisterController(
          std::string(spec.name),
          [kind = spec.kind, id = std::string(spec.name)](const auto& c) {
            if (ParametersFor(c.local_controllers, id).empty()) {
              return Controller(kind, c);
            }
            return Controller(kind, ResolveBuiltinParameters(c, id, false));
          },
          spec.capabilities,
          [id = std::string(spec.name)](const auto& c, const auto& parameters) {
            if (!parameters.empty()) {
              static_cast<void>(ResolveBuiltinParameters(c, id, false));
            }
          });
    }
    return std::shared_ptr<const AlgorithmRegistry>(std::move(result));
  }();
  return registry;
}
}  // namespace navigation2d
