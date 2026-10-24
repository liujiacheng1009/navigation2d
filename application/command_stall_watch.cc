#include "navigation2d/application/command_stall_watch.h"

#include <algorithm>
#include <cmath>

namespace navigation2d {
void CommandStallWatch::Reset() {
  initialized_ = false;
  cycles_ = 0;
}

void CommandStallWatch::Begin(const StallObservation& o) {
  initialized_ = true;
  started_s_ = o.timestamp_s;
  previous_s_ = o.timestamp_s;
  anchor_ = o.pose;
  progress_m_ = o.route_progress_m;
  heading_error_rad_ = std::abs(o.heading_error_rad);
  measured_distance_m_ = 0.;
  measured_rotation_rad_ = 0.;
  cycles_ = 0;
}

bool CommandStallWatch::MotionObserved(const StallObservation& o) {
  const double dt = std::clamp(o.timestamp_s - previous_s_, 0., .20);
  previous_s_ = o.timestamp_s;
  measured_distance_m_ += std::abs(o.measured_velocity.linear) * dt;
  measured_rotation_rad_ += std::abs(o.measured_velocity.angular) * dt;
  const double displacement = (o.pose.translation() - anchor_.translation()).norm();
  const double angle = Yaw(o.pose) - Yaw(anchor_);
  const double yaw_change = std::abs(std::atan2(std::sin(angle), std::cos(angle)));
  const double route_delta = o.route_progress_m - progress_m_;
  const double heading_improvement = heading_error_rad_ - std::abs(o.heading_error_rad);
  // Pose/route integration tolerates quantized or delayed encoder samples.
  // Rotation counts only when translation was not requested.
  return displacement >= .025 || route_delta >= .02 ||
         (!o.translation_expected && (yaw_change >= .03 || heading_improvement >= .03)) ||
         measured_distance_m_ >= .015 || (!o.translation_expected && measured_rotation_rad_ >= .03);
}

bool CommandStallWatch::Observe(const StallObservation& o) {
  if (!o.commanded) {
    Reset();
    return false;
  }
  if (!initialized_) {
    Begin(o);
    return false;
  }
  if (MotionObserved(o)) {
    Reset();
    return false;
  }
  ++cycles_;
  // A planning-time jump alone must not expire a command streak before enough
  // actual control cycles ran. Preserve both the cycle and elapsed-time gates.
  const int min_cycles =
      std::max(8, static_cast<int>(std::ceil(.9 / std::max(.02, control_period_s_))));
  return cycles_ >= min_cycles && o.timestamp_s - started_s_ >= .9;
}
}  // namespace navigation2d
