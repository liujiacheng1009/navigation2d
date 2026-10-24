#include "navigation2d/ros/exploration/motion_output.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
namespace {
void Require(bool condition) {
  if (!condition) {
    throw std::runtime_error("motion output contract failed");
  }
}
struct Shutdown {
  ~Shutdown() { rclcpp::shutdown(); }
};
}  // namespace
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  Shutdown shutdown;
  auto node = std::make_shared<rclcpp::Node>("navigation2d_motion_output_test");
  const auto unique =
      static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
  const std::string topic = "/navigation2d_test/motion_output_" + std::to_string(unique);
  std::optional<geometry_msgs::msg::Twist> received;
  auto subscription = node->create_subscription<geometry_msgs::msg::Twist>(
      topic, 10, [&](geometry_msgs::msg::Twist::ConstSharedPtr value) { received = *value; });
  auto output = navigation2d::exploration_ros::MakeRosMotionOutput(*node, topic);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto wait = [&](auto ready) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!ready() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(ready());
  };
  wait([&] { return subscription->get_publisher_count() == 1; });
  for (const auto command : std::vector<navigation2d::Twist2d>{{.18, -.7}, {}, {-.12, .9}}) {
    received.reset();
    output->Send(command);
    wait([&] { return received.has_value(); });
    Require(received->linear.x == command.linear && received->angular.z == command.angular);
    Require(received->linear.y == 0. && received->linear.z == 0. && received->angular.x == 0. &&
            received->angular.y == 0.);
  }
}
