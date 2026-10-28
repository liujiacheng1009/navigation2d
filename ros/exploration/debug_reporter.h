#pragma once

#include <functional>

#include "navigation2d/exploration/frontier_explorer.h"
#include "navigation2d/ros/exploration/debug_output.h"
#include "navigation2d/ros/exploration/debug_paths.h"
#include "std_msgs/msg/string.hpp"

namespace navigation2d::exploration_ros {
struct RouteDebugView {
  const Path& route;
  double origin_x, origin_y, progress_m;
};
struct TourDebugView {
  const nav_msgs::msg::OccupancyGrid* map;
  double robot_x, robot_y;
  const ExplorationGoal& active;
  const std::deque<ExplorationGoal>& goals;
};

// Owns the display cursor and message sequencing, never mission state. Called
// on the node's single event thread. Clock is evaluated once, lazily at the
// original message-stamping point (after input-map copies for PlanningMap).
class DebugReporter {
 public:
  using Clock = std::function<rclcpp::Time()>;
  void ClearPaths(DebugReportSink& sink, const Clock& clock);
  void Paths(DebugReportSink& sink, const RouteDebugView& route, const TourDebugView& tour,
             const Clock& clock);
  void PlanningMap(DebugReportSink& sink, const nav_msgs::msg::OccupancyGrid& map,
                   const Pose2d& robot_local, const Clock& clock) const;
  void Frontiers(DebugReportSink& sink, FrontierExplorer& explorer) const;
  const DebugPaths& cursor() const { return paths_; }

 private:
  DebugPaths paths_;
};
}  // namespace navigation2d::exploration_ros
