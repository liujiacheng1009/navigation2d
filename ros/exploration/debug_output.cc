#include "navigation2d/ros/exploration/debug_output.h"

#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

namespace navigation2d::exploration_ros {
namespace {
class RosDebugOutput final : public DebugReportSink {
 public:
  RosDebugOutput(rclcpp::Node& node, const std::string& prefix) {
    path_publisher_ = node.create_publisher<nav_msgs::msg::Path>(
        prefix + "/navigation/path", rclcpp::QoS(1).transient_local().reliable());
    local_trajectory_publisher_ = node.create_publisher<nav_msgs::msg::Path>(
        prefix + "/navigation/local_trajectory", rclcpp::QoS(1).transient_local().reliable());
    tour_publisher_ = node.create_publisher<nav_msgs::msg::Path>(
        prefix + "/navigation/tour", rclcpp::QoS(1).transient_local().reliable());
    carrot_publisher_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(
        prefix + "/navigation/carrot", rclcpp::QoS(1).transient_local().reliable());
    frontiers_publisher_ = node.create_publisher<geometry_msgs::msg::PoseArray>(
        prefix + "/exploration/frontier_goals", rclcpp::QoS(1).transient_local().reliable());
    frontier_debug_publisher_ = node.create_publisher<std_msgs::msg::String>(
        prefix + "/exploration/frontier_debug", rclcpp::QoS(1).transient_local().reliable());
    planning_input_publisher_ = node.create_publisher<nav_msgs::msg::OccupancyGrid>(
        prefix + "/exploration/global_planning_input", rclcpp::QoS(1).transient_local().reliable());
    inflated_costmap_publisher_ = node.create_publisher<nav_msgs::msg::OccupancyGrid>(
        prefix + "/exploration/global_costmap_inflated",
        rclcpp::QoS(1).transient_local().reliable());
  }
  void SendPath(navigation2d::exploration_ros::DebugPathChannel channel,
                const nav_msgs::msg::Path& message) override {
    using navigation2d::exploration_ros::DebugPathChannel;
    switch (channel) {
      case DebugPathChannel::kGlobal:
        path_publisher_->publish(message);
        return;
      case DebugPathChannel::kLocal:
        local_trajectory_publisher_->publish(message);
        return;
      case DebugPathChannel::kTour:
        tour_publisher_->publish(message);
        return;
    }
    throw std::invalid_argument("unknown debug path channel");
  }
  void SendMap(navigation2d::exploration_ros::DebugMapChannel channel,
               const nav_msgs::msg::OccupancyGrid& message) override {
    using navigation2d::exploration_ros::DebugMapChannel;
    switch (channel) {
      case DebugMapChannel::kPlanningInput:
        planning_input_publisher_->publish(message);
        return;
      case DebugMapChannel::kInflated:
        inflated_costmap_publisher_->publish(message);
        return;
    }
    throw std::invalid_argument("unknown debug map channel");
  }
  void SendCarrot(const geometry_msgs::msg::PoseStamped& message) override {
    carrot_publisher_->publish(message);
  }
  void SendFrontiers(const geometry_msgs::msg::PoseArray& message) override {
    frontiers_publisher_->publish(message);
  }
  void SendFrontierDebug(const std_msgs::msg::String& message) override {
    frontier_debug_publisher_->publish(message);
  }

 private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_trajectory_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tour_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr frontiers_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr frontier_debug_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr planning_input_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_costmap_publisher_;
};
}  // namespace
std::unique_ptr<DebugReportSink> MakeRosDebugOutput(rclcpp::Node& node, const std::string& prefix) {
  return std::make_unique<RosDebugOutput>(node, prefix);
}
}  // namespace navigation2d::exploration_ros
