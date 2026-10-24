#include "navigation2d/ros/exploration/motion_output.h"

#include <utility>

#include "geometry_msgs/msg/twist.hpp"
namespace navigation2d::exploration_ros {
namespace {
class MotionPublisher final : public exploration::MotionOutput {
 public:
  explicit MotionPublisher(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher)
      : publisher_(std::move(publisher)) {}
  void Send(Twist2d command) override {
    geometry_msgs::msg::Twist message;
    message.linear.x = command.linear;
    message.angular.z = command.angular;
    publisher_->publish(message);
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};
}  // namespace
std::unique_ptr<exploration::MotionOutput> MakeRosMotionOutput(rclcpp::Node& node,
                                                               const std::string& topic) {
  return std::make_unique<MotionPublisher>(
      node.create_publisher<geometry_msgs::msg::Twist>(topic, 10));
}
}  // namespace navigation2d::exploration_ros
