#include "navigation2d/application/navigation_configuration.h"

#include <cassert>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "navigation2d/application/algorithm_registry.h"
#include "navigation2d/application/navigation_system.h"
#include "navigation2d/control/regulated_pure_pursuit.h"
#include "navigation2d/planning/dijkstra_planner.h"
#include "navigation2d/test/navigation_configuration_fixture.h"
namespace {
using namespace navigation2d;
template <class Operation>
void Reject(Operation&& operation, const std::string& message = {}) {
  bool rejected = false;
  try {
    operation();
  } catch (const std::exception& error) {
    rejected = true;
    assert(message.empty() || std::string(error.what()).find(message) != std::string::npos);
  }
  assert(rejected);
}
}  // namespace
int main() {
  using namespace navigation2d;
  test::NavigationConfigurationFixture fixture;
  for (const auto* global : {"dijkstra", "astar", "theta_star", "state_lattice"}) {
    for (const auto* local : {"rpp", "dwa", "mppi", "mpc"}) {
      auto root = fixture.Fresh();
      root["selection"]["planner"] = global;
      root["selection"]["controller"] = local;
      const auto loaded = NavigationConfig::Load(fixture.Write(root));
      assert(loaded.planner == global && loaded.controller == local);
    }
  }
  int planner_creations = 0, controller_creations = 0;
  auto registry = std::make_shared<AlgorithmRegistry>();
  registry->RegisterPlanner("custom_grid", [&](const auto& config) {
    ++planner_creations;
    return std::make_unique<DijkstraPlanner>(config.robot_radius);
  });
  registry->RegisterController("custom_follow", [&](const auto& config) {
    ++controller_creations;
    return std::make_unique<RegulatedPurePursuit>(config);
  });
  auto custom = fixture.Fresh();
  custom["selection"]["planner"] = "custom_grid";
  custom["selection"]["controller"] = "custom_follow";
  const auto file = fixture.Write(custom);
  Reject([&] { static_cast<void>(NavigationConfig::Load(file)); },
         "unknown planner or controller selection");
  const auto config = LoadNavigationConfig(file, *registry);
  assert(config.planner == "custom_grid" && config.controller == "custom_follow");
  assert(planner_creations == 0 && controller_creations == 0);
  NavigationSystem navigation(config, fixture.map(), registry);
  navigation.SetGoal(MakePose2d(3., 1.5, 0.));
  const auto state = navigation.ComputeCommand(MakePose2d(1., 1.5, 0.), {}, 0.);
  assert(planner_creations > 0 && controller_creations > 0);
  assert(navigation.planner_id() == "custom_grid" && navigation.controller_id() == "custom_follow");
  assert(state.global_path_length_m > 1.8);
  assert(std::isfinite(state.command.linear) && std::isfinite(state.command.angular));
  custom["selection"]["planner"] = "missing_grid";
  Reject([&] { static_cast<void>(LoadNavigationConfig(fixture.Write(custom), *registry)); },
         "unknown planner: missing_grid");
  custom["selection"]["planner"] = "custom_grid";
  custom["selection"]["controller"] = "missing_follow";
  Reject([&] { static_cast<void>(LoadNavigationConfig(fixture.Write(custom), *registry)); },
         "unknown controller: missing_follow");
  for (int fault = 0; fault < 6; ++fault) {
    auto root = fixture.Fresh();
    switch (fault) {
      case 0:
        root["unexpected"] = true;
        break;
      case 1:
        root.remove("dwa");
        break;
      case 2:
        root["map"]["resolution"] = std::numeric_limits<double>::quiet_NaN();
        break;
      case 3:
        root["costmap"]["inflation_radius"] = 0.;
        break;
      case 4:
        root["mpc"]["solver"] = "missing_solver";
        break;
      case 5:
        root["costmap"]["footprint"][0] = 1;
        break;
    }
    const auto invalid = fixture.Write(root);
    Reject([&] { static_cast<void>(NavigationConfig::Load(invalid)); });
    Reject(
        [&] { static_cast<void>(LoadNavigationConfig(invalid, *AlgorithmRegistry::Builtins())); });
  }
}
