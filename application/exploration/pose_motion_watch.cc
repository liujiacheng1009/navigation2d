#include "navigation2d/application/exploration/pose_motion_watch.h"

#include <cmath>
namespace navigation2d::exploration {
void PoseMotionWatch::Observe(double x, double y, std::int64_t now_ns) {
  if (!initialized_ || std::hypot(x - x_, y - y_) > .12) {
    x_ = x;
    y_ = y;
    stamp_ns_ = now_ns;
    initialized_ = true;
  }
}
bool PoseMotionWatch::Stalled(const PoseStallObservation& observation) const {
  if (!initialized_) {
    return false;
  }
  const auto& o = observation;
  const bool open_slow = !o.entry_drive && !o.tip_wedged && o.front_clear;
  const double minimum_goal_s = o.entry_drive ? 14.0 : (open_slow ? 8.0 : 4.0);
  if ((o.now_ns - o.goal_started_ns) / 1e9 < minimum_goal_s) {
    return false;
  }
  const double pose_limit_s = o.entry_drive ? 10.0 : (open_slow ? 7.5 : 3.5);
  const bool pose_stuck = (o.now_ns - stamp_ns_) / 1e9 >= pose_limit_s;
  const double path_limit_s = o.entry_drive ? 12.0 : (open_slow ? 7.5 : 3.5);
  const double path_age_s = (o.now_ns - o.path_progress_stamp_ns) / 1e9;
  const bool path_stuck =
      o.path_progress_m < .05 || (o.path_progress_stamp_ns > 0 && path_age_s > path_limit_s);
  // A mouth traversal that still advances must not be preempted by the tour.
  if (o.entry_drive && o.path_progress_m >= .15 && o.path_progress_stamp_ns > 0 &&
      path_age_s < 10.0) {
    return false;
  }
  return pose_stuck && path_stuck;
}
}  // namespace navigation2d::exploration
