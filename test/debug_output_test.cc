#include "navigation2d/ros/exploration/debug_output.h"

#include <chrono>
#include <source_location>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace {
void Require(bool value, const std::source_location& at = std::source_location::current()) {
  if (!value) {
    throw std::runtime_error("debug publication contract failed at " + std::to_string(at.line()));
  }
}
template <class Predicate>
void SpinUntil(rclcpp::Executor& executor, Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  Require(predicate());
}
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  using namespace navigation2d::exploration_ros;
  const std::string prefix = "/debug_output_test";
  auto publisher = std::make_shared<rclcpp::Node>("debug_output_publisher");
  auto subscriber = std::make_shared<rclcpp::Node>("debug_output_subscriber");
  auto output = MakeRosDebugOutput(*publisher, prefix);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher);
  executor.add_node(subscriber);
  const auto qos = rclcpp::QoS(1).transient_local().reliable();
  int received = 0;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  for (const auto* suffix :
       {"/navigation/path", "/navigation/local_trajectory", "/navigation/tour"}) {
    subscriptions.push_back(subscriber->create_subscription<nav_msgs::msg::Path>(
        prefix + suffix, qos, [&](const nav_msgs::msg::Path& message) {
          Require(message.header.frame_id == "test_map" && message.poses.size() == 1);
          ++received;
        }));
  }
  for (const auto* suffix :
       {"/exploration/global_planning_input", "/exploration/global_costmap_inflated"}) {
    subscriptions.push_back(subscriber->create_subscription<nav_msgs::msg::OccupancyGrid>(
        prefix + suffix, qos, [&](const nav_msgs::msg::OccupancyGrid& message) {
          Require(message.info.width == 2 && message.data == std::vector<std::int8_t>({0, -1}));
          ++received;
        }));
  }
  subscriptions.push_back(subscriber->create_subscription<geometry_msgs::msg::PoseStamped>(
      prefix + "/navigation/carrot", qos, [&](const geometry_msgs::msg::PoseStamped& message) {
        Require(message.pose.position.x == 1.25);
        ++received;
      }));
  subscriptions.push_back(subscriber->create_subscription<geometry_msgs::msg::PoseArray>(
      prefix + "/exploration/frontier_goals", qos,
      [&](const geometry_msgs::msg::PoseArray& message) {
        Require(message.poses.size() == 2);
        ++received;
      }));
  subscriptions.push_back(subscriber->create_subscription<std_msgs::msg::String>(
      prefix + "/exploration/frontier_debug", qos, [&](const std_msgs::msg::String& message) {
        Require(message.data == "{\"test\":true}");
        ++received;
      }));
  SpinUntil(executor, [&] {
    for (const auto& subscription : subscriptions) {
      if (subscription->get_publisher_count() != 1) {
        return false;
      }
    }
    return true;
  });
  for (const auto& subscription : subscriptions) {
    const auto publishers =
        subscriber->get_publishers_info_by_topic(subscription->get_topic_name());
    Require(publishers.size() == 1);
    const auto profile = publishers.front().qos_profile().get_rmw_qos_profile();
    // This RMW reports discovered depth as zero. Check advertised transport
    // policies here; a reader requesting depth 10 verifies retained data below.
    Require(profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE &&
            profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }
  nav_msgs::msg::Path path;
  path.header.frame_id = "test_map";
  path.poses.resize(1);
  for (auto channel :
       {DebugPathChannel::kGlobal, DebugPathChannel::kLocal, DebugPathChannel::kTour}) {
    output->SendPath(channel, path);
  }
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 2;
  map.data = {0, -1};
  output->SendMap(DebugMapChannel::kPlanningInput, map);
  output->SendMap(DebugMapChannel::kInflated, map);
  geometry_msgs::msg::PoseStamped carrot;
  carrot.pose.position.x = 1.25;
  output->SendCarrot(carrot);
  geometry_msgs::msg::PoseArray frontiers;
  frontiers.poses.resize(2);
  output->SendFrontiers(frontiers);
  std_msgs::msg::String debug;
  debug.data = "{\"test\":true}";
  output->SendFrontierDebug(debug);
  SpinUntil(executor, [&] { return received == 8; });
  for (int sequence = 1; sequence <= 3; ++sequence) {
    path.header.stamp.sec = sequence;
    output->SendPath(DebugPathChannel::kGlobal, path);
  }
  bool latched = false;
  auto late = subscriber->create_subscription<nav_msgs::msg::Path>(
      prefix + "/navigation/path", rclcpp::QoS(10).transient_local().reliable(),
      [&](const nav_msgs::msg::Path& message) {
        Require(message.header == path.header);
        latched = true;
      });
  SpinUntil(executor, [&] { return latched; });
  rclcpp::shutdown();
}
