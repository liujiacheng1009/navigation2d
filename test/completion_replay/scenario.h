#pragma once
#include <sys/resource.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "navigation2d/test/replay/explorer_scenario.h"
namespace navigation2d::test {
// Direct phase characterization: all ROS outputs remain captured; never spin.
// Tick first initializes the mission/home using the same input guard as normal.
template <class Node>
int RunCompletionReplay(int argc, char** argv) {
  const auto wall_start = std::chrono::steady_clock::now();
  const bool benchmark = argc > 1 && std::string(argv[1]) == "--benchmark";
  if (benchmark) {
    --argc;
    ++argv;
  }
  std::vector<std::pair<std::string, double>> timings;
  std::ostringstream trace;
  trace << std::setprecision(17) << '[';
  bool first = true;
  for (const std::string name : {"empty", "residual", "blocked", "finish"}) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node>();
    ReplayWorld world;
    auto map = world.map;
    map.info.resolution = .05;
    map.info.width = 100;
    map.info.height = 60;
    map.data.assign(6000, name == "empty" ? 0 : 100);
    if (name != "empty") {
      for (int row = 8; row < 52; ++row) {
        for (int col = 8; col < 56; ++col) {
          map.data[row * 100 + col] = 0;
        }
      }
      for (int row = 16; row < 44; ++row) {
        for (int col = 56; col < 90; ++col) {
          map.data[row * 100 + col] = -1;
        }
      }
    }
    node->replay_.probe_time_ = rclcpp::Time(1000000000LL, RCL_ROS_TIME);
    node->map_ = map;
    node->map_revision_ = 17;
    node->explorer_.UpdateMap({100, 60, map.info.resolution, 0., 0., map.data});
    geometry_msgs::msg::PoseStamped pose;
    pose.pose = ReplayPose(name == "blocked" ? .15 : 1.2, 1.5, 0.);
    node->ReplayObservePose(pose);
    node->scan_ = world.Observe();
    node->velocity_stamp_ = node->replay_.probe_time_;
    node->Tick();
    // Expire the bootstrap wait before directly invoking the phase.
    node->replay_.probe_time_ = rclcpp::Time(10000000000LL, RCL_ROS_TIME);
    node->velocity_stamp_ = node->replay_.probe_time_;
    const auto begin = std::chrono::steady_clock::now();
    node->StartCompletingProbe();
    if ((name == "residual" || name == "finish") && node->ReplayPhase() != 1) {
      throw std::runtime_error("required completing phase was not reached: " + name);
    }
    if (name == "finish") {
      node->FinishCompleting();
    }
    timings.emplace_back(
        name, std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - begin)
                  .count());
    if (!first) {
      trace << ',';
    }
    first = false;
    trace << "{\"case\":" << std::quoted(name) << ",\"phase\":" << node->ReplayPhase()
          << ",\"can_probe\":" << node->ReplayCanCompleteProbe()
          << ",\"finished\":" << node->ReplayFinished()
          << ",\"pending\":" << node->ReplayPendingEntry()
          << ",\"locked\":" << node->ReplayLockedEntry()
          << ",\"attempts\":" << node->ReplayEntryAttempts()
          << ",\"navigation\":" << node->ReplayHasNavigation()
          << ",\"completed\":" << node->explorer_.completed_goals()
          << ",\"failed\":" << node->explorer_.failed_goals()
          << ",\"publishes\":" << node->replay_.probe_publishes_
          << ",\"commands\":" << node->replay_.probe_commands_
          << ",\"vx\":" << node->replay_.probe_command_.linear.x
          << ",\"w\":" << node->replay_.probe_command_.angular.z
          << ",\"overlay\":" << node->replay_.probe_overlay_hash_
          << ",\"paths\":" << node->replay_.path_messages.value()
          << ",\"state\":" << std::quoted(node->last_state_)
          << ",\"message\":" << std::quoted(node->last_message_) << '}';
    // Finish may shut down its context; each scenario owns a fresh one.
    node.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
  trace << ']';
  std::cout << std::setprecision(17);
  if (benchmark) {
    std::cout << "{\"trace\":" << trace.str();
    for (const auto& [name, elapsed] : timings) {
      std::cout << ',' << std::quoted(name + "_us") << ':' << elapsed;
    }
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    std::cout
        << ",\"peak_rss_kb\":" << usage.ru_maxrss << ",\"wall_seconds\":"
        << std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count()
        << "}\n";
  } else {
    std::cout << trace.str() << '\n';
  }
  return 0;
}
}  // namespace navigation2d::test
