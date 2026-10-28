#pragma once
#include "navigation2d/control/collision_monitor_types.h"

namespace navigation2d::collision_internal {
// Hysteresis only. The caller bypasses this stage on source timeout or
// disabled filtering, preserving the pending trigger/release counts.
// Consecutive non-clear requests share a trigger count even if their kind changes.
class CollisionLatch {
 public:
  CollisionMonitorAction Update(CollisionMonitorAction requested, int trigger_cycles,
                                int release_cycles) {
    if (requested != CollisionMonitorAction::kNone) {
      ++trigger_count_;
      release_count_ = 0;
      if (trigger_count_ >= trigger_cycles) {
        action_ = requested;
      }
    } else {
      trigger_count_ = 0;
      ++release_count_;
      if (release_count_ >= release_cycles) {
        action_ = CollisionMonitorAction::kNone;
      }
    }
    return action_;
  }

 private:
  int trigger_count_ = 0, release_count_ = 0;
  CollisionMonitorAction action_ = CollisionMonitorAction::kNone;
};
}  // namespace navigation2d::collision_internal
