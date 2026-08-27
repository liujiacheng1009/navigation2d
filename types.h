#pragma once

#include <vector>

namespace navigation2d {

struct Pose2d { double x = 0.; double y = 0.; double yaw = 0.; };
struct Velocity { double linear = 0.; double angular = 0.; };
struct Point2d { double x = 0.; double y = 0.; };
struct LaserScan {
  double angle_min = 0.;
  double angle_increment = 0.;
  double range_min = 0.;
  double range_max = 0.;
  std::vector<double> ranges;
};
struct PointCloud2d {
  double range_max = 0.;
  std::vector<Point2d> points;  // Sensor-frame hit endpoints.
};
using Path = std::vector<Pose2d>;

}  // namespace navigation2d
