#include "navigation2d/planning/motion_primitives.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace navigation2d::planning_internal {
namespace {
double Normalize(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

MotionPrimitive MakeTranslation(int bin, int bin_delta, int direction, int yaw_bins,
                                double resolution, double requested_length) {
  const double bin_angle = 2. * M_PI / yaw_bins;
  const double start_yaw = bin * bin_angle;
  const double yaw_change = bin_delta * bin_angle;
  const double chord_heading = start_yaw + .5 * yaw_change;
  int dx = static_cast<int>(std::lround(direction * requested_length *
                                         std::cos(chord_heading) / resolution));
  int dy = static_cast<int>(std::lround(direction * requested_length *
                                         std::sin(chord_heading) / resolution));
  if (dx == 0 && dy == 0) {
    dx = static_cast<int>(std::lround(direction * std::cos(start_yaw)));
    dy = static_cast<int>(std::lround(direction * std::sin(start_yaw)));
  }
  const double endpoint_x = dx * resolution, endpoint_y = dy * resolution;
  const double length = std::hypot(endpoint_x, endpoint_y);
  MotionPrimitive primitive{bin, (bin + bin_delta + yaw_bins) % yaw_bins, dx, dy,
      direction > 0 ? MotionType::kForward : MotionType::kReverse,
      length, yaw_change, {}};
  const int samples = std::max(2, static_cast<int>(std::ceil(length / (.5 * resolution))));
  for (int sample = 1; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / samples;
    // Constant-curvature interpolation, scaled so the discretized endpoint is exact.
    const double smooth = yaw_change == 0. ? ratio :
        std::sin(.5 * ratio * std::abs(yaw_change)) /
        std::sin(.5 * std::abs(yaw_change));
    const double heading = start_yaw + .5 * ratio * yaw_change;
    const double radial = smooth * length;
    primitive.samples.push_back(MakePose2d(
        direction * radial * std::cos(heading), direction * radial * std::sin(heading),
        Normalize(start_yaw + ratio * yaw_change)));
  }
  primitive.samples.back() = MakePose2d(endpoint_x, endpoint_y,
      primitive.end_yaw_bin * bin_angle);
  return primitive;
}

MotionPrimitive MakeRotation(int bin, int delta, int yaw_bins) {
  const double bin_angle = 2. * M_PI / yaw_bins;
  MotionPrimitive primitive{bin, (bin + delta + yaw_bins) % yaw_bins, 0, 0,
                            MotionType::kRotate, 0., delta * bin_angle, {}};
  primitive.samples.push_back(MakePose2d(0., 0., primitive.end_yaw_bin * bin_angle));
  return primitive;
}
}  // namespace

DifferentialDrivePrimitiveSet::DifferentialDrivePrimitiveSet(
    int yaw_bins, double resolution, double primitive_length, bool allow_reverse)
    : yaw_bins_(yaw_bins), primitives_(yaw_bins), incoming_(yaw_bins) {
  if (yaw_bins < 8 || yaw_bins % 2 != 0 || resolution <= 0. ||
      primitive_length < resolution)
    throw std::invalid_argument("invalid differential drive lattice parameters");
  for (int bin = 0; bin < yaw_bins; ++bin) {
    for (int delta : {-1, 0, 1})
      primitives_[bin].push_back(MakeTranslation(
          bin, delta, 1, yaw_bins, resolution, primitive_length));
    if (primitive_length > 1.5 * resolution)
      primitives_[bin].push_back(MakeTranslation(bin, 0, 1, yaw_bins, resolution, resolution));
    primitives_[bin].push_back(MakeRotation(bin, -1, yaw_bins));
    primitives_[bin].push_back(MakeRotation(bin, 1, yaw_bins));
    if (allow_reverse) for (int delta : {-1, 0, 1})
      primitives_[bin].push_back(MakeTranslation(
          bin, delta, -1, yaw_bins, resolution, primitive_length));
    if (allow_reverse && primitive_length > 1.5 * resolution)
      primitives_[bin].push_back(MakeTranslation(bin, 0, -1, yaw_bins, resolution, resolution));
  }
  for (int bin = 0; bin < yaw_bins; ++bin)
    for (std::size_t index = 0; index < primitives_[bin].size(); ++index)
      incoming_[primitives_[bin][index].end_yaw_bin].push_back({bin, index});
}

const std::vector<DifferentialDrivePrimitiveSet::Reference>&
DifferentialDrivePrimitiveSet::IntoYawBin(int yaw_bin) const {
  if (yaw_bin < 0 || yaw_bin >= yaw_bins_) throw std::out_of_range("invalid yaw bin");
  return incoming_[yaw_bin];
}

const std::vector<MotionPrimitive>& DifferentialDrivePrimitiveSet::FromYawBin(int yaw_bin) const {
  if (yaw_bin < 0 || yaw_bin >= yaw_bins_) throw std::out_of_range("invalid yaw bin");
  return primitives_[yaw_bin];
}

const MotionPrimitive* DifferentialDrivePrimitiveSet::Find(
    int yaw_bin, int dx, int dy, int end_yaw_bin) const {
  const auto& candidates = FromYawBin(yaw_bin);
  const auto found = std::find_if(candidates.begin(), candidates.end(), [&](const auto& primitive) {
    return primitive.dx_cells == dx && primitive.dy_cells == dy &&
           primitive.end_yaw_bin == end_yaw_bin;
  });
  return found == candidates.end() ? nullptr : &*found;
}

}  // namespace navigation2d::planning_internal
