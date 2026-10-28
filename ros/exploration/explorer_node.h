#pragma once
#include <memory>

#include "navigation2d/application/algorithm_registry.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
namespace navigation2d::exploration_ros {
// Registry is shared immutable assembly input. The same instance validates
// all purpose profiles and constructs actual navigation sessions. Null is
// rejected; default CLI behavior uses builtins. Construction does not spin.
[[nodiscard]] std::shared_ptr<rclcpp::Node> MakeAutonomousExplorer(
    std::shared_ptr<const AlgorithmRegistry> algorithms = AlgorithmRegistry::Builtins(),
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
}  // namespace navigation2d::exploration_ros
