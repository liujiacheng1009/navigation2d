#pragma once
#include <deque>
#include <utility>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigation2d/exploration/frontier_types.h"
#include "navigation2d/geometry/path_2d.h"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"

namespace navigation2d::exploration_ros {
// Message construction and display cursor only. Does not publish or change a
// mission. Borrowed map/path inputs need only remain alive during each call.
class DebugPaths {
 public:
  nav_msgs::msg::Path Global(const Path& route, double origin_x, double origin_y,
                             const rclcpp::Time& stamp) const;
  nav_msgs::msg::Path Local(const nav_msgs::msg::Path& global, double progress_m);
  geometry_msgs::msg::PoseStamped Carrot(const std_msgs::msg::Header& header) const;
  std::pair<nav_msgs::msg::Path, geometry_msgs::msg::PoseArray> Tour(
      const std_msgs::msg::Header& header, const nav_msgs::msg::OccupancyGrid* map, double robot_x,
      double robot_y, const ExplorationGoal& active,
      const std::deque<ExplorationGoal>& goals) const;
  void Clear() { last_carrot_valid_ = false; }
  double x() const { return last_carrot_x_; }
  double y() const { return last_carrot_y_; }
  double yaw() const { return last_carrot_yaw_; }
  bool valid() const { return last_carrot_valid_; }

 private:
  double last_carrot_x_ = 0., last_carrot_y_ = 0., last_carrot_yaw_ = 0.;
  bool last_carrot_valid_ = false;
};
}  // namespace navigation2d::exploration_ros
