#include <cassert>
#include <memory>

#include "navigation2d/application/algorithm_registry.h"

namespace {
class Planner final : public navigation2d::GlobalPlanner {
 public:
  navigation2d::Path Plan(const navigation2d::LayeredCostmap&, const navigation2d::Pose2d&,
                          const navigation2d::Pose2d&) const override {
    return {};
  }
};
class Controller final : public navigation2d::LocalController {
 public:
  navigation2d::Twist2d Compute(
      const navigation2d::Path&, const navigation2d::Pose2d&, navigation2d::Twist2d,
      const navigation2d::LayeredCostmap&,
      const std::vector<navigation2d::PredictedObstacle>&) const override {
    return {};
  }
  bool CollisionImminent(const navigation2d::Pose2d&, navigation2d::Twist2d,
                         const navigation2d::LayeredCostmap&) const override {
    return false;
  }
};
}  // namespace
int main() {
  navigation2d::AlgorithmRegistry registry;
  int creations = 0;
  registry.RegisterPlanner("external_global", [&](const auto&) {
    ++creations;
    return std::make_unique<Planner>();
  });
  registry.RegisterController("external_local", [&](const auto&) {
    ++creations;
    return std::make_unique<Controller>();
  });
  navigation2d::NavigationConfig config;
  config.planner = "external_global";
  config.controller = "external_local";
  registry.Validate(config);
  assert(creations == 0);
  assert(registry.CreatePlanner(config));
  assert(registry.CreateController(config));
  assert(creations == 2);
}
