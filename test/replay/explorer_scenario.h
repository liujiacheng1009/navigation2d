#pragma once

#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "navigation2d/test/replay/replay_world.h"
#include "navigation2d/test/replay/ros_capture.h"
#include "rclcpp/rclcpp.hpp"

namespace navigation2d::test {
inline geometry_msgs::msg::Pose ReplayPose(double x, double y, double yaw) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.z = std::sin(yaw * .5);
  pose.orientation.w = std::cos(yaw * .5);
  return pose;
}

template <class Node>
int RunExplorerReplay(int argc, char** argv) {
  const auto wall_start = std::chrono::steady_clock::now();
  const bool stuck = argc > 1 && std::string(argv[1]) == "stuck";
  if (argc > 1) {
    --argc;
    ++argv;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Node>();
  ReplayWorld world;
  geometry_msgs::msg::Twist applied;
  std::vector<double> timings;
  std::ostringstream trace;
  trace.precision(17);
  bool first = true;
  for (int step = 0; step < 600 && !node->ReplayFinished(); ++step) {
    node->replay_.probe_time_ = rclcpp::Time(1000000000LL + 60000000LL * step, RCL_ROS_TIME);
    applied = world.Advance(node->replay_.probe_command_, stuck, step);
    const double x = world.x, y = world.y, yaw = world.yaw;
    geometry_msgs::msg::PoseStamped pose;
    pose.pose = ReplayPose(x, y, yaw);
    node->ReplayObservePose(pose);
    node->ReplayObserveVelocity({applied.linear.x, applied.angular.z}, node->replay_.probe_time_);
    const auto scan = world.Observe();
    node->ReplayObserveScan(scan);
    if (step % 5 == 0) {
      node->ReplayObserveMap(world.map);
      node->explorer_.UpdateMap({80, 60, world.res, 0., 0., world.map.data});
      ++node->map_revision_;
    }
    if (node->ReplayHasNavigation()) {
      node->FeedScan();
    }
    const auto begin = std::chrono::steady_clock::now();
    node->Tick();
    timings.push_back(
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - begin)
            .count());
    if (!first) {
      trace << ',';
    }
    first = false;
    trace << '[' << step << ',' << node->ReplayPhase() << ',' << x << ',' << y << ','
          << yaw << ',' << node->replay_.probe_command_.linear.x << ','
          << node->replay_.probe_command_.angular.z << ',' << node->explorer_.KnownCells() << ','
          << node->explorer_.completed_goals() << ',' << node->explorer_.failed_goals() << ','
          << node->replay_.probe_commands_ << ',' << node->replay_.probe_publishes_ << ','
          << node->ReplayEntryAttempts() << ',' << node->ReplayReturnEscapes() << ','
          << node->replay_.probe_overlay_hash_ << ']';
  }
  std::cout << "{\"path_messages_hash\":" << node->replay_.path_messages.value() << ",\"trace\":["
            << trace.str() << "],\"tick_us\":[";
  for (std::size_t i = 0; i < timings.size(); ++i) {
    if (i) {
      std::cout << ',';
    }
    std::cout << timings[i];
  }
  auto ordered = timings;
  std::sort(ordered.begin(), ordered.end());
  std::cout << "],\"tick_p50_us\":"
            << ordered[static_cast<std::size_t>(std::ceil(ordered.size() * .5)) - 1];
  std::cout << ",\"tick_p95_us\":"
            << ordered[static_cast<std::size_t>(std::ceil(ordered.size() * .95)) - 1]
            << ",\"tick_p99_us\":"
            << ordered[static_cast<std::size_t>(std::ceil(ordered.size() * .99)) - 1];
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  std::cout << ",\"peak_rss_kb\":" << usage.ru_maxrss << ",\"wall_seconds\":"
            << std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count()
            << "}\n";
  node.reset();
  rclcpp::shutdown();
  return 0;
}

}  // namespace navigation2d::test
