#pragma once

#include <Eigen/Core>
#include <vector>

namespace navigation2d {

struct LaserScan {
  double angle_min = 0.;
  double angle_increment = 0.;
  double range_min = 0.;
  double range_max = 0.;
  std::vector<double> ranges;
};

struct PointCloud2d {
  double range_max = 0.;
  std::vector<Eigen::Vector2d> points;
};

}  // namespace navigation2d
