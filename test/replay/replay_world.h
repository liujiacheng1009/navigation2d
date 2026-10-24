#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navigation2d/geometry/robot_geometry.h"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace navigation2d::test {
// Fixed dual-room map, point-collision differential drive and 64-ray lidar.
// This is a deterministic scheduling fixture, not a chassis collision oracle.
class ReplayWorld {
 public:
  nav_msgs::msg::OccupancyGrid map;
  std::vector<int> truth;
  double res = 0.;
  double x = .65, y = 1.5, yaw = 0.;
  ReplayWorld() {
    map.info.width = 80;
    map.info.height = 60;
    map.info.resolution = .05F;
    res = map.info.resolution;
    truth.assign(4800, 0);
    for (int y = 0; y < 60; ++y) {
      for (int x = 0; x < 80; ++x) {
        if (x < 2 || x >= 78 || y < 2 || y >= 58 || (x >= 39 && x <= 40 && (y < 23 || y > 36))) {
          truth[y * 80 + x] = 1;
        }
      }
    }
    map.data.assign(4800, -1);
  }
  geometry_msgs::msg::Twist Advance(geometry_msgs::msg::Twist requested, bool stuck, int step) {
    auto applied = requested;
    if (stuck && step >= 180 && step < 260) {
      applied = geometry_msgs::msg::Twist{};
    }
    double next_yaw = yaw + applied.angular.z * .06;
    double nx = x, ny = y;
    if (std::abs(applied.angular.z) < 1e-9) {
      nx += applied.linear.x * std::cos(yaw) * .06;
      ny += applied.linear.x * std::sin(yaw) * .06;
    } else {
      double radius = applied.linear.x / applied.angular.z;
      nx += radius * (std::sin(next_yaw) - std::sin(yaw));
      ny -= radius * (std::cos(next_yaw) - std::cos(yaw));
    }
    int cx = static_cast<int>(std::floor(nx / res)), cy = static_cast<int>(std::floor(ny / res));
    if (cx < 0 || cy < 0 || cx >= 80 || cy >= 60 || truth[cy * 80 + cx]) {
      applied = geometry_msgs::msg::Twist{};
    } else {
      x = nx;
      y = ny;
      yaw = std::atan2(std::sin(next_yaw), std::cos(next_yaw));
    }
    return applied;
  }
  sensor_msgs::msg::LaserScan Observe() {
    sensor_msgs::msg::LaserScan scan;
    scan.angle_min = -static_cast<float>(std::acos(-1.));
    scan.angle_increment = static_cast<float>(2. * std::acos(-1.) / 64.);
    scan.range_min = .02F;
    scan.range_max = 3.F;
    const double sx = x + navigation2d::kLidarForwardOffset() * std::cos(yaw),
                 sy = y + navigation2d::kLidarForwardOffset() * std::sin(yaw);
    for (int beam = 0; beam < 64; ++beam) {
      const double angle = yaw + scan.angle_min + beam * static_cast<double>(scan.angle_increment);
      float hit = std::numeric_limits<float>::infinity();
      for (double distance = .02; distance < 3.; distance += .025) {
        int bx = static_cast<int>(std::floor((sx + distance * std::cos(angle)) / res));
        int by = static_cast<int>(std::floor((sy + distance * std::sin(angle)) / res));
        if (bx < 0 || by < 0 || bx >= 80 || by >= 60) {
          hit = static_cast<float>(distance);
          break;
        }
        if (truth[by * 80 + bx]) {
          map.data[by * 80 + bx] = 100;
          hit = static_cast<float>(distance);
          break;
        }
        map.data[by * 80 + bx] = 0;
      }
      scan.ranges.push_back(hit);
    }
    return scan;
  }
};
}  // namespace navigation2d::test
