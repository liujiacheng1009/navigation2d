#pragma once
#include <sys/resource.h>

#include <algorithm>
#include <bit>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "navigation2d/geometry/twist_2d.h"
#include "rclcpp/rclcpp.hpp"
namespace navigation2d::test {
inline const std::string& MotionBenchmarkTopic() {
  static const std::string topic =
      "/navigation2d_test/motion_benchmark_" +
      std::to_string(
          static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count()));
  return topic;
}
// A matched test-only reader exercises the real ROS publisher API.
// Round-trip payload correctness belongs to motion_output_test.
template <class Send>
void MeasureMotionOutput(const rclcpp::Node::SharedPtr& node, Send send) {
  using Clock = std::chrono::steady_clock;
  auto reader = node->create_subscription<geometry_msgs::msg::Twist>(
      MotionBenchmarkTopic(), 2048, [](geometry_msgs::msg::Twist::ConstSharedPtr) {});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto discovery_deadline = Clock::now() + std::chrono::seconds(3);
  while (reader->get_publisher_count() != 1 && Clock::now() < discovery_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (reader->get_publisher_count() != 1) {
    throw std::runtime_error("motion benchmark reader missing");
  }
  for (int i = 0; i < 16; ++i) {
    send({});
  }
  executor.spin_some();
  std::vector<double> samples;
  samples.reserve(2048);
  std::uint64_t signature = 1469598103934665603ULL;
  const auto wall_begin = Clock::now();
  for (int i = 0; i < 2048; ++i) {
    const Twist2d command{.01 * (i % 21 - 10), .02 * (i % 37 - 18)};
    const auto begin = Clock::now();
    send(command);
    samples.push_back(std::chrono::duration<double, std::micro>(Clock::now() - begin).count());
    signature = (signature ^ std::bit_cast<std::uint64_t>(command.linear)) * 1099511628211ULL;
    signature = (signature ^ std::bit_cast<std::uint64_t>(command.angular)) * 1099511628211ULL;
    if (i % 32 == 31) {
      executor.spin_some();
    }
  }
  executor.spin_some();
  const double wall_seconds = std::chrono::duration<double>(Clock::now() - wall_begin).count();
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) {
    throw std::runtime_error("cannot read motion benchmark RSS");
  }
  std::sort(samples.begin(), samples.end());
  const auto percentile = [&](double p) {
    return samples[static_cast<std::size_t>(p * (samples.size() - 1))];
  };
  std::cout << std::setprecision(17) << "{\"commands\":2048,\"input_signature\":" << signature
            << ",\"tick_p50_us\":" << percentile(.50) << ",\"tick_p95_us\":" << percentile(.95)
            << ",\"tick_p99_us\":" << percentile(.99) << ",\"wall_seconds\":" << wall_seconds
            << ",\"peak_rss_kb\":" << usage.ru_maxrss << "}\n";
}
}  // namespace navigation2d::test
