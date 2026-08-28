#pragma once

#include <vector>

#include "navigation2d/geometry/pose_2d.h"

namespace navigation2d::planning_internal {

enum class MotionType { kForward, kReverse, kRotate };

struct MotionPrimitive {
  int start_yaw_bin = 0;
  int end_yaw_bin = 0;
  int dx_cells = 0;
  int dy_cells = 0;
  MotionType type = MotionType::kForward;
  double length_m = 0.;
  double yaw_change = 0.;
  std::vector<Pose2d> samples;  // Relative to the source cell centre.
};

class DifferentialDrivePrimitiveSet {
 public:
  struct Reference { int start_yaw_bin; std::size_t index; };
  DifferentialDrivePrimitiveSet(int yaw_bins, double resolution,
                                double primitive_length, bool allow_reverse);
  int yaw_bins() const { return yaw_bins_; }
  const std::vector<MotionPrimitive>& FromYawBin(int yaw_bin) const;
  const MotionPrimitive* Find(int yaw_bin, int dx, int dy, int end_yaw_bin) const;
  const std::vector<Reference>& IntoYawBin(int yaw_bin) const;

 private:
  int yaw_bins_;
  std::vector<std::vector<MotionPrimitive>> primitives_;
  std::vector<std::vector<Reference>> incoming_;
};

}  // namespace navigation2d::planning_internal
