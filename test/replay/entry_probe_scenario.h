#pragma once
#include <sys/resource.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "navigation2d/test/replay/explorer_scenario.h"
namespace navigation2d::test {
// Direct entry-flow characterization with real maps, explorer and navigation.
// Publishers remain captured, and no executor or vehicle receives commands.
template <class Node>
int RunEntryProbeReplay(int argc, char** argv) {
  const auto wall_start = std::chrono::steady_clock::now();
  const bool benchmark = std::string(argv[1]) == "entry-benchmark";
  std::vector<std::pair<std::string, double>> timings;
  std::ostringstream trace;
  trace.precision(17);
  --argc;
  ++argv;
  rclcpp::init(argc, argv);

  trace << '[';
  bool first = true;
  for (const auto* scenario : {"approach", "unavailable", "probe", "inside", "cooling"}) {
    auto node = std::make_shared<Node>();
    const auto initial_publishes = node->replay_.probe_publishes_;
    const auto initial_commands = node->replay_.probe_commands_;
    const std::string name = scenario;
    nav_msgs::msg::OccupancyGrid map;
    map.info.resolution = .05;
    map.info.width = name == "approach" ? 140 : 50;
    map.info.height = name == "approach" ? 60 : 30;
    map.data.assign(map.info.width * map.info.height, name == "approach" ? 0 : 100);
    if (name != "approach" && name != "unavailable") {
      for (int row = 4; row < 26; ++row) {
        for (int col = 2; col < 16; ++col) {
          map.data[row * 50 + col] = 0;
        }
        for (int col = 21; col < 29; ++col) {
          map.data[row * 50 + col] = 0;
        }
      }
      for (int row = 8; row < 22; ++row) {
        for (int col = 16; col < 21; ++col) {
          map.data[row * 50 + col] = -1;
        }
      }
    }
    node->ReplayObserveMap(map);
    node->map_revision_ = 17;
    node->explorer_.UpdateMap({static_cast<int>(map.info.width), static_cast<int>(map.info.height),
                               map.info.resolution, 0., 0., map.data});
    ExplorationGoal mouth;
    mouth.x = name == "approach" ? 3. : .35;
    mouth.y = name == "approach" ? 1.5 : .75;
    mouth.frontier_x = name == "approach" ? 4. : .8;
    mouth.frontier_y = mouth.y;
    geometry_msgs::msg::PoseStamped pose;
    pose.pose = ReplayPose(name == "inside" ? 1.35 : (name == "approach" ? 1. : .4), mouth.y, 0.);
    node->ReplayObservePose(pose);
    node->ReplaySetPendingEntry(mouth);
    if (name == "cooling") {
      if constexpr (requires { node->DemoteViewpointForSeconds(mouth, 14.); }) {
        node->DemoteViewpointForSeconds(mouth, 14.);
      } else {
        node->recently_demoted_ = mouth;
        node->recently_demoted_until_ = node->now() + rclcpp::Duration::from_seconds(14.);
      }
    }
    const auto begin = std::chrono::steady_clock::now();
    const bool driving = node->StartEntryProbe();
    timings.emplace_back(
        name, std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - begin)
                  .count());
    if (!first) {
      trace << ',';
    }
    first = false;
    trace << "{\"case\":\"" << name << "\",\"driving\":" << driving
          << ",\"pending\":" << node->ReplayPendingEntry()
          << ",\"locked\":" << node->ReplayLockedEntry()
          << ",\"attempts\":" << node->ReplayEntryAttempts()
          << ",\"navigation\":" << node->ReplayHasNavigation()
          << ",\"completed\":" << node->explorer_.completed_goals()
          << ",\"failed\":" << node->explorer_.failed_goals()
          << ",\"publishes\":" << (node->replay_.probe_publishes_ - initial_publishes)
          << ",\"commands\":" << (node->replay_.probe_commands_ - initial_commands)
          << ",\"overlay\":" << node->replay_.probe_overlay_hash_
          << ",\"paths\":" << node->replay_.path_messages.value() << '}';
  }
  trace << ']';
  if (benchmark) {
    std::cout << "{\"trace\":" << trace.str();
    for (const auto& [name, elapsed] : timings) {
      std::cout << ",\"" << name << "_us\":" << elapsed;
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
  rclcpp::shutdown();
  return 0;
}
}  // namespace navigation2d::test
