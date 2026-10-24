#pragma once

#include <cstdint>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace navigation2d::test {
// In-memory sinks only: no captured command is sent to a ROS publisher.
// Only the two explicit measured status fields are normalized for comparison.
struct ReplayOutputs {
  rclcpp::Time probe_time_{1000000000LL, RCL_ROS_TIME};
  geometry_msgs::msg::Twist probe_command_;
  int probe_commands_ = 0, probe_publishes_ = 0;
  void ProbeCommand(const geometry_msgs::msg::Twist& value) {
    probe_command_ = value;
    ++probe_commands_;
  }
  std::uint64_t probe_overlay_hash_ = 1469598103934665603ULL;
  void ProbeHash(const std::string& bytes) {
    for (unsigned char c : bytes) {
      probe_overlay_hash_ ^= c;
      probe_overlay_hash_ *= 1099511628211ULL;
    }
    probe_overlay_hash_ ^= 255;
    probe_overlay_hash_ *= 1099511628211ULL;
  }
  void ProbePublish(const std_msgs::msg::String& msg) {
    ++probe_publishes_;
    auto stable = msg.data;
    for (const auto* key : {"\"controller_solve_us\":", "\"planner_elapsed_s\":"}) {
      const auto begin = stable.find(key);
      if (begin != std::string::npos) {
        const auto value = begin + std::string(key).size();
        const auto end = stable.find_first_of(",}", value);
        stable.replace(value, end - value, "0");
      }
    }
    ProbeHash(stable);
  }
  void ProbePublish(const nav_msgs::msg::OccupancyGrid& msg) {
    ++probe_publishes_;
    ProbeHash(msg.header.frame_id);
    ProbeHash(std::to_string(msg.info.width));
    ProbeHash(std::to_string(msg.info.height));
    ProbeHash(std::to_string(msg.info.resolution));
    ProbeHash(std::to_string(msg.info.origin.position.x));
    ProbeHash(std::to_string(msg.info.origin.position.y));
    ProbeHash(std::string(reinterpret_cast<const char*>(msg.data.data()), msg.data.size()));
  }
  template <class T>
  void ProbePublish(const T&) {
    ++probe_publishes_;
  }
};
}  // namespace navigation2d::test
