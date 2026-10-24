#pragma once
#include <cstdint>
namespace navigation2d::exploration {
struct PoseStallObservation {
  std::int64_t now_ns, goal_started_ns, path_progress_stamp_ns;
  double path_progress_m;
  bool entry_drive, tip_wedged, front_clear;
};
// Accumulated pose motion is independent of encoder velocity and path progress.
// Coordinates are world-frame metres; all stamps use one mission clock.
class PoseMotionWatch {
 public:
  void Observe(double x, double y, std::int64_t now_ns);
  bool Stalled(const PoseStallObservation& observation) const;
  void Debounce(std::int64_t now_ns) { stamp_ns_ = now_ns; }

 private:
  bool initialized_ = false;
  double x_ = 0., y_ = 0.;
  std::int64_t stamp_ns_ = 0;
};
}  // namespace navigation2d::exploration
