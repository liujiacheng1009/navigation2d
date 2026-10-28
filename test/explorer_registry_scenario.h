#pragma once
#include <cassert>
#include <iostream>
#include <memory>

#include "navigation2d/application/exploration/navigation_profile.h"
#include "navigation2d/test/navigation_plugin_fixture.h"
#include "rclcpp/rclcpp.hpp"
namespace navigation2d::test {
// Actual node source + captured output seam; no executor, simulation or motion.
template <typename Node>
int RunExplorerRegistryProbe(int argc, char** argv) {
  rclcpp::init(argc, argv);
  NavigationPluginFixture fixture;
  auto registry = std::make_shared<AlgorithmRegistry>(*AlgorithmRegistry::Builtins());
  registry->RegisterPlanner(fixture.config.planner, [&](const auto&) {
    return std::make_unique<RequestPlanner>(fixture.calls);
  });
  registry->RegisterController(fixture.config.controller, [&](const auto&) {
    return std::make_unique<RequestController>(fixture.calls);
  });
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("explore.global_planner", fixture.config.planner),
       rclcpp::Parameter("explore.local_controller", fixture.config.controller)});
  {
    Node node(registry, options);
    const auto& selected =
        node.parameters_.navigation_profiles.For(exploration::NavigationPurpose::kExplore);
    assert(selected.global == fixture.config.planner &&
           selected.local == fixture.config.controller);
    const auto config = exploration::BuildNavigationProfile({.resolution = .1}, selected);
    node.navigation_.Begin(config, fixture.map_path, MakePose2d(2., 1., 0.), {},
                           {.heading_policy = selected.heading_policy});
    const auto probe = node.navigation_.Probe(MakePose2d(1., 1., 0.), {}, 0.);
    assert(probe.has_route() && fixture.calls.plans == 1 && fixture.calls.controls == 1);
    assert(node.replay_.probe_commands_ == 0);
  }
  rclcpp::shutdown();
  std::cout << "custom ROS startup registry reached planner/controller; zero motion publications\n";
  return 0;
}
}  // namespace navigation2d::test
