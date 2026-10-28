#include <cassert>
#include <memory>
#include <stdexcept>
#include <string>

#include "navigation2d/ros/exploration/explorer_node.h"
#include "navigation2d/test/navigation_plugin_fixture.h"
#include "rclcpp/rclcpp.hpp"
int main(int argc, char** argv) {
  using namespace navigation2d;
  rclcpp::init(argc, argv);
  test::NavigationPluginFixture fixture;
  auto registry = std::make_shared<AlgorithmRegistry>(*AlgorithmRegistry::Builtins());
  registry->RegisterPlanner("startup_global", [&](const auto&) {
    return std::make_unique<test::RequestPlanner>(fixture.calls);
  });
  registry->RegisterController("startup_local", [&](const auto&) {
    return std::make_unique<test::RequestController>(fixture.calls);
  });
  rclcpp::NodeOptions options;
  options.enable_rosout(false).start_parameter_services(false).start_parameter_event_publisher(
      false);
  options.parameter_overrides({rclcpp::Parameter("explore.global_planner", "startup_global"),
                               rclcpp::Parameter("explore.local_controller", "startup_local")});
  std::weak_ptr<const AlgorithmRegistry> lifetime = registry;
  auto node = exploration_ros::MakeAutonomousExplorer(registry, options);
  assert(node->get_parameter("explore.global_planner").as_string() == "startup_global");
  assert(node->get_parameter("explore.local_controller").as_string() == "startup_local");
  assert(fixture.calls.plans == 0 && fixture.calls.controls == 0);  // No spin/input/task.
  registry.reset();
  assert(!lifetime.expired());
  node.reset();
  assert(lifetime.expired());
  try {
    static_cast<void>(exploration_ros::MakeAutonomousExplorer({}, options));
    assert(false);
  } catch (const std::invalid_argument&) {
  }
  try {
    static_cast<void>(
        exploration_ros::MakeAutonomousExplorer(AlgorithmRegistry::Builtins(), options));
    assert(false);
  } catch (const std::runtime_error& error) {
    assert(std::string(error.what()) == "unknown planner: startup_global");
  }
  rclcpp::shutdown();
}
