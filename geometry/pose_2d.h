#pragma once

#include <Eigen/Core>
#include <sophus/se2.hpp>

namespace navigation2d {

using Pose2d = Sophus::SE2d;

inline Pose2d MakePose2d(double x, double y, double yaw) {
  return Pose2d(yaw, Eigen::Vector2d(x, y));
}
inline double X(const Pose2d& pose) { return pose.translation().x(); }
inline double Y(const Pose2d& pose) { return pose.translation().y(); }
inline double Yaw(const Pose2d& pose) { return pose.so2().log(); }

}  // namespace navigation2d
