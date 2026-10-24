#include "geometry_msgs/msg/twist.hpp"
#include "navigation2d/test/motion_output_benchmark.h"
#include "rclcpp/rclcpp.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("motion_output_baseline");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(
      navigation2d::test::MotionBenchmarkTopic(), 10);
  navigation2d::test::MeasureMotionOutput(node, [&](navigation2d::Twist2d command) {
    // Exact former node conversion/publication, independent of the new port.
    geometry_msgs::msg::Twist message;
    message.linear.x = command.linear;
    message.angular.z = command.angular;
    publisher->publish(message);
  });
  rclcpp::shutdown();
}
