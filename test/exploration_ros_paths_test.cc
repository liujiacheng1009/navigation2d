#include <cmath>
#include <limits>
#include <stdexcept>

#include "navigation2d/ros/exploration/debug_paths.h"
#include "navigation2d/ros/exploration/map_observation.h"
#include "navigation2d/ros/exploration/scan_observation.h"
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
  nav_msgs::msg::OccupancyGrid raw;
  raw.info.width = 2;
  raw.info.height = 2;
  raw.info.resolution = .05f;
  raw.info.origin.position.x = -1.;
  raw.info.origin.position.y = 2.;
  raw.data = {-1, 49, 50, 100};
  const auto copied = ConvertOccupancyGrid(raw);
  Require(copied.width == 2 && copied.height == 2 && copied.resolution == raw.info.resolution &&
          copied.origin_x == -1. && copied.origin_y == 2. && copied.cells == raw.data);
  raw.data[0] = 100;
  Require(copied.cells[0] == -1);
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
  // Conversion retains borrowed evidence and independently owned navigation data.
  Require(!ScanEvidence(nullptr));
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -.4F;
  scan.angle_increment = .2F;
  scan.range_min = .1F;
  scan.range_max = 3.F;
  scan.ranges = {1.F, std::numeric_limits<float>::infinity(),
                 std::numeric_limits<float>::quiet_NaN()};
  const auto evidence = ScanEvidence(&scan);
  const auto owned = CopyScan(scan);
  Require(evidence && evidence->ranges.data() == scan.ranges.data());
  Require(owned.angle_min == scan.angle_min && owned.angle_increment == scan.angle_increment &&
          owned.range_min == scan.range_min && owned.range_max == scan.range_max);
  Require(std::isinf(owned.ranges[1]) && std::isnan(owned.ranges[2]));
  scan.ranges[0] = 2.F;
  Require(evidence->ranges[0] == 2.F && owned.ranges[0] == 1.F);
  // Map-frame translation and the observed occupancy threshold are independent
  // of the planning-local route coordinates.
  map.data.assign(100, -1);
  exploration::PlanningSnapshot snapshot;
  const Path shifted{MakePose2d(0., 0., 0.), MakePose2d(8., 0., 0.)};
  const auto cuts = [&] {
    return ObservedRouteCutsWall(&map, shifted, .5, .5, snapshot, std::nullopt, std::nullopt,
                                 false);
  };
  Require(!cuts());
  map.data[3] = 49;
  Require(!cuts());
  map.data[3] = 50;
  Require(cuts());
  Require(!ObservedRouteCutsWall(nullptr, shifted, 0., 0., snapshot, std::nullopt, std::nullopt,
                                 false));
  Require(!ObservedRouteCutsWall(&map, {}, 0., 0., snapshot, std::nullopt, std::nullopt, false));
}
