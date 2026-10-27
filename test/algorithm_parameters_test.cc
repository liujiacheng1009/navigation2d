#include <cassert>
#include <memory>
#include <stdexcept>
#include <string>

#include "navigation2d/application/algorithm_registry.h"
#include "navigation2d/application/navigation_configuration.h"
#include "navigation2d/application/typed_algorithm_registration.h"
#include "navigation2d/configuration/builtin_parameters.h"
#include "navigation2d/test/navigation_configuration_fixture.h"

namespace {
template <class F>
void Reject(F operation) {
  bool failed = false;
  try {
    operation();
  } catch (const std::exception&) {
    failed = true;
  }
  assert(failed);
}
struct ExternalConfig {
  int horizon;
};
class Planner final : public navigation2d::GlobalPlanner {
 public:
  explicit Planner(ExternalConfig config) : horizon(config.horizon) {}
  navigation2d::Path Plan(const navigation2d::LayeredCostmap&, const navigation2d::Pose2d&,
                          const navigation2d::Pose2d&) const override {
    return {};
  }
  int horizon;
};
}  // namespace
int main() {
  using namespace navigation2d;
  test::NavigationConfigurationFixture fixture;
  auto root = fixture.Fresh();
  root["global_planners"]["state_lattice"]["yaw_bins"] = 48;
  root["local_controllers"]["dwa"]["horizon"] = 1.25;
  root["local_controllers"]["rpp"]["lookahead_time"] = 2.25;
  const auto config = NavigationConfig::Load(fixture.Write(root));
  assert(config.lattice_yaw_bins != 48);
  assert(ResolveBuiltinParameters(config, "state_lattice", true).lattice_yaw_bins == 48);
  assert(ResolveBuiltinParameters(config, "dwa", false).dwa_horizon == 1.25);
  assert(ResolveBuiltinParameters(config, "rpp", false).lookahead_time == 2.25);
  assert(ResolveBuiltinParameters(config, "rpp", false).dwa_horizon == config.dwa_horizon);
  AlgorithmRegistry::Builtins()->Validate(config);
  AlgorithmRegistry::Builtins()->Validate(config, {.in_place_rotation = true});
  auto reverse = config;
  reverse.planner = "state_lattice";
  reverse.controller = "mppi";
  Reject([&] { AlgorithmRegistry::Builtins()->Validate(reverse, {.reverse_paths = true}); });
  reverse.controller = "rpp";
  Reject([&] { AlgorithmRegistry::Builtins()->Validate(reverse, {.reverse_paths = true}); });
  reverse.controller = "mppi";
  reverse.planner = "astar";
  Reject([&] { AlgorithmRegistry::Builtins()->Validate(reverse, {.reverse_paths = true}); });
  for (const auto* value : {"nan", "inf", "1.5junk", "-1"}) {
    auto invalid = config;
    invalid.local_controllers["dwa"]["horizon"] = value;
    Reject([&] { AlgorithmRegistry::Builtins()->Validate(invalid); });
  }
  auto invalid = config;
  invalid.local_controllers["dwa"]["unknown_option"] = "1";
  Reject([&] { AlgorithmRegistry::Builtins()->Validate(invalid); });
  invalid = config;
  invalid.local_controllers["typo"]["horizon"] = "1";
  Reject([&] { AlgorithmRegistry::Builtins()->Validate(invalid); });
  invalid = config;
  invalid.global_planners["state_lattice"]["yaw_bins"] = "7";
  Reject([&] { AlgorithmRegistry::Builtins()->Validate(invalid); });
  invalid = config;
  invalid.local_controllers["rpp"]["control_period"] = ".01";
  Reject([&] { AlgorithmRegistry::Builtins()->Validate(invalid); });
  auto malformed = fixture.Fresh();
  malformed["local_controllers"]["rpp"]["lookahead_time"].push_back(1);
  Reject([&] { NavigationConfig::Load(fixture.Write(malformed)); });

  AlgorithmRegistry registry;
  int constructions = 0;
  RegisterTypedPlanner(
      registry, "external",
      [](const NavigationConfig&, const AlgorithmParameters& p) {
        if (p.size() != 1 || !p.contains("horizon")) {
          throw std::invalid_argument("horizon required");
        }
        auto value = ReadAlgorithmParameter<int>(p.at("horizon"), "external.horizon");
        if (value < 1) {
          throw std::invalid_argument("positive horizon required");
        }
        return ExternalConfig{value};
      },
      [&](ExternalConfig c) {
        ++constructions;
        return std::make_unique<Planner>(c);
      });
  // A backend-free local registration lets validation exercise typed planner parsing.
  registry.RegisterController("unused",
                              [](const NavigationConfig&) -> std::unique_ptr<LocalController> {
                                throw std::logic_error("validation must not construct");
                              });
  NavigationConfig custom;
  custom.planner = "external";
  custom.controller = "unused";
  Reject([&] { registry.Validate(custom); });
  custom.global_planners["external"]["horizon"] = "12";
  registry.Validate(custom);
  assert(constructions == 0);
  auto first = registry.CreatePlanner(custom);
  auto second = registry.CreatePlanner(custom);
  assert(constructions == 2 && first.get() != second.get());
  assert(dynamic_cast<Planner&>(*first).horizon == 12);
  auto custom_yaml = fixture.Fresh();
  custom_yaml["selection"]["planner"] = "external";
  custom_yaml["selection"]["controller"] = "unused";
  custom_yaml["global_planners"]["external"]["horizon"] = 19;
  auto loaded = LoadNavigationConfig(fixture.Write(custom_yaml), registry);
  assert(dynamic_cast<Planner&>(*registry.CreatePlanner(loaded)).horizon == 19);
  assert(constructions == 3);
  custom.global_planners["external"]["horizon"] = "0";
  Reject([&] { registry.Validate(custom); });
  assert(constructions == 3);
}
