#include <cmath>
#include <stdexcept>

#include "navigation2d/ros/exploration/debug_paths.h"
#include "navigation2d/ros/exploration/map_observation.h"
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("ROS path contract failed");
  }
}
}  // namespace
int main() {
  using namespace navigation2d;
  using namespace navigation2d::exploration_ros;
  DebugPaths display;
  const Path route{MakePose2d(0., 0., 0.), MakePose2d(1., 0., .2), MakePose2d(2., 0., .4)};
  const auto global = display.Global(route, -2., 3., rclcpp::Time(42, 0, RCL_ROS_TIME));
  Require(global.header.frame_id == "map" && global.header.stamp.sec == 42);
  Require(global.poses.size() == 3 && global.poses.front().pose.position.x == -2.);
  const auto local = display.Local(global, .5);
  Require(local.poses.size() == 3 && display.valid() && display.x() == -1.5);
  const auto carrot = display.Carrot(global.header);
  Require(carrot.header == global.header && carrot.pose.position.x == -1.5);
  const auto past = display.Local(global, 8.);
  Require(past.poses.size() == 1 && display.x() == -2.);
  nav_msgs::msg::Path empty;
  Require(display.Local(empty, 0.).poses.empty() && !display.valid());
  nav_msgs::msg::Path singleton;
  singleton.poses.push_back(global.poses.front());
  Require(display.Local(singleton, 0.).poses.size() == 1 && display.valid());
  display.Local(global, .5);
  display.Clear();
  Require(!display.valid() && display.x() == -1.5);  // Clearing keeps last coordinates.
  Require(display.Carrot(global.header).pose.orientation.w == 1.);
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 10;
  map.info.height = 10;
  map.info.resolution = 1.;
  map.data.assign(100, -1);
  Require(!SegmentCutsWall(&map, .5, .5, 8.5, .5));
  Require(SegmentCutsWall(&map, .5, .5, 10.5, .5));
  map.data[3] = 49;
  Require(!SegmentCutsWall(&map, .5, .5, 8.5, .5));
  map.data[3] = 50;
  Require(SegmentCutsWall(&map, .5, .5, 8.5, .5));
  ExplorationGoal active;
  active.x = 1.5;
  active.y = .5;
  active.frontier_x = 2.;
  active.frontier_y = .5;
  auto other = active;
  other.x = 8.5;
  const auto [tour, frontiers] = display.Tour(global.header, &map, .5, .5, active, {active, other});
  Require(tour.poses.size() == 1 && frontiers.poses.size() == 1);
  Require(tour.poses.front().pose.position.x == active.x);
  Require(!SegmentCutsWall(nullptr, 0., 0., 1., 1.));
}
