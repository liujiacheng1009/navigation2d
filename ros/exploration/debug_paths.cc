#include "navigation2d/ros/exploration/debug_paths.h"

#include <algorithm>
#include <cmath>

#include "navigation2d/ros/exploration/map_observation.h"
namespace navigation2d::exploration_ros {
namespace {
double Yaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Pose PoseFromXyYaw(double x, double y, double yaw) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.z = std::sin(yaw / 2.);
  pose.orientation.w = std::cos(yaw / 2.);
  return pose;
}

}  // namespace
using Goal = ExplorationGoal;
nav_msgs::msg::Path DebugPaths::Global(const Path& route, double origin_x, double origin_y,
                                       const rclcpp::Time& stamp) const {
  nav_msgs::msg::Path global;
  global.header.stamp = stamp;
  global.header.frame_id = "map";
  global.poses.reserve(route.size());
  for (const auto& pose : route) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = global.header;
    stamped.pose = PoseFromXyYaw(navigation2d::X(pose) + origin_x, navigation2d::Y(pose) + origin_y,
                                 navigation2d::Yaw(pose));
    global.poses.push_back(stamped);
  }
  return global;
}
nav_msgs::msg::Path DebugPaths::Local(const nav_msgs::msg::Path& global, double progress_m) {
  nav_msgs::msg::Path local;
  local.header = global.header;
  // Lookahead band from current progress (Nav2-style local plan window).
  double traveled = 0.;
  bool started = false;
  last_carrot_valid_ = false;
  for (std::size_t i = 1; i < global.poses.size(); ++i) {
    const auto& a = global.poses[i - 1].pose.position;
    const auto& b = global.poses[i].pose.position;
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    const double prev = traveled;
    traveled += seg;
    if (!started && traveled + 1e-6 >= progress_m) {
      started = true;
      local.poses.push_back(global.poses[i - 1]);
      const double remain = progress_m - prev;
      const double t = seg > 1e-6 ? std::clamp(remain / seg, 0., 1.) : 0.;
      last_carrot_x_ = a.x + t * (b.x - a.x);
      last_carrot_y_ = a.y + t * (b.y - a.y);
      last_carrot_yaw_ = Yaw(global.poses[i].pose.orientation);
      last_carrot_valid_ = true;
    }
    if (started) {
      local.poses.push_back(global.poses[i]);
      if (traveled - progress_m > 2.5) {
        break;
      }
    }
  }
  if (local.poses.empty() && !global.poses.empty()) {
    local.poses.push_back(global.poses.front());
    last_carrot_x_ = global.poses.front().pose.position.x;
    last_carrot_y_ = global.poses.front().pose.position.y;
    last_carrot_yaw_ = Yaw(global.poses.front().pose.orientation);
    last_carrot_valid_ = true;
  }
  return local;
}
geometry_msgs::msg::PoseStamped DebugPaths::Carrot(const std_msgs::msg::Header& header) const {
  geometry_msgs::msg::PoseStamped carrot;
  carrot.header = header;
  if (last_carrot_valid_) {
    carrot.pose = PoseFromXyYaw(last_carrot_x_, last_carrot_y_, last_carrot_yaw_);
  }
  return carrot;
}
std::pair<nav_msgs::msg::Path, geometry_msgs::msg::PoseArray> DebugPaths::Tour(
    const std_msgs::msg::Header& header, const nav_msgs::msg::OccupancyGrid* map, double robot_x,
    double robot_y, const ExplorationGoal& active, const std::deque<ExplorationGoal>& goals) const {
  nav_msgs::msg::Path tour;
  tour.header = header;
  geometry_msgs::msg::PoseArray frontiers;
  frontiers.header = header;
  // Tour is a Path message, so the UI draws straight edges between poses.
  // Never chain goals that cut a live mapped wall — that looked like a
  // through-wall global plan even when Navigation2D routed around.
  bool published_active = false;
  double prev_x = robot_x;
  double prev_y = robot_y;
  if (std::isfinite(active.x) && std::isfinite(active.y)) {
    frontiers.poses.push_back(PoseFromXyYaw(active.frontier_x, active.frontier_y, active.yaw));
    // Skip the straight robot→goal chord when it pierces a mapped wall; the
    // real route is on /navigation/path (GLOBAL). Publishing the chord made
    // the UI look like 穿墙.
    if (!SegmentCutsWall(map, prev_x, prev_y, active.x, active.y)) {
      geometry_msgs::msg::PoseStamped stamped;
      stamped.header = tour.header;
      stamped.pose = PoseFromXyYaw(active.x, active.y, active.yaw);
      tour.poses.push_back(stamped);
      prev_x = active.x;
      prev_y = active.y;
      published_active = true;
    }
  }
  for (const Goal& goal : goals) {
    if (published_active && std::hypot(goal.x - active.x, goal.y - active.y) < .05 &&
        std::hypot(goal.frontier_x - active.frontier_x, goal.frontier_y - active.frontier_y) <
            .05) {
      published_active = false;  // skip only the matching front entry once
      continue;
    }
    if (SegmentCutsWall(map, prev_x, prev_y, goal.x, goal.y)) {
      break;
    }
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = tour.header;
    stamped.pose = PoseFromXyYaw(goal.x, goal.y, goal.yaw);
    tour.poses.push_back(stamped);
    prev_x = goal.x;
    prev_y = goal.y;
  }
  return {std::move(tour), std::move(frontiers)};
}
}  // namespace navigation2d::exploration_ros
