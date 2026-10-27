#pragma once
#include <functional>
#include <memory>
#include <string>

#include "navigation2d/application/algorithm_registry.h"
#include "navigation2d/application/navigation_system.h"
#include "navigation2d/test/algorithm_contract_fixture.h"

namespace navigation2d::test {
struct PluginCalls {
  int plans = 0, controls = 0;
  std::function<GlobalPlanResult(const GlobalPlanRequest&)> plan;
  std::function<LocalControlResult(const LocalControlRequest&)> control;
};
class RequestPlanner final : public GlobalPlanner {
 public:
  explicit RequestPlanner(PluginCalls& calls) : calls_(calls) {}

 protected:
  GlobalPlanResult PlanRequest(const GlobalPlanRequest& request) const override {
    ++calls_.plans;
    return calls_.plan(request);
  }

 private:
  PluginCalls& calls_;
};
class RequestController final : public LocalController {
 public:
  explicit RequestController(PluginCalls& calls) : calls_(calls) {}
  bool CollisionImminent(const Pose2d&, Twist2d, const LayeredCostmap&) const override {
    return false;
  }

 protected:
  LocalControlResult ComputeRequest(const LocalControlRequest& request) const override {
    ++calls_.controls;
    return calls_.control(request);
  }

 private:
  PluginCalls& calls_;
};
struct NavigationPluginFixture {
  AlgorithmContractFixture files;
  PluginCalls calls;
  NavigationConfig config;
  std::string map_path;
  NavigationPluginFixture() {
    static_cast<void>(files.Map());
    map_path = (files.directory.path() / "map.json").string();
    config.map_resolution = .1;
    config.planner = "request_planner";
    config.controller = "request_controller";
    config.smoother_enabled = false;
    config.collision_monitor_enabled = false;
    calls.plan = [](const auto& request) {
      return GlobalPlanResult{GlobalPlanStatus::kSuccess,
                              {request.start, request.goal},
                              {std::string(request.frame_id), request.costmap.revision(),
                               PathHeadingSemantics::kForwardTangent},
                              {}};
    };
    calls.control = [](const auto&) {
      return LocalControlResult{LocalControlStatus::kSuccess, {.12, 0.}, {}};
    };
  }
  std::shared_ptr<const AlgorithmRegistry> Registry(PlannerCapabilities planner = {},
                                                    ControllerCapabilities controller = {}) {
    auto registry = std::make_shared<AlgorithmRegistry>();
    registry->RegisterPlanner(
        config.planner, [this](const auto&) { return std::make_unique<RequestPlanner>(calls); },
        planner);
    registry->RegisterController(
        config.controller,
        [this](const auto&) { return std::make_unique<RequestController>(calls); }, controller);
    return registry;
  }
};
}  // namespace navigation2d::test
