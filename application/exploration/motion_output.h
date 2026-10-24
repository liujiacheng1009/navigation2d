#pragma once
#include "navigation2d/geometry/twist_2d.h"
namespace navigation2d::exploration {
// Synchronous transport boundary for every exploration motion command
// (linear m/s, angular rad/s). Safety
// and mission decisions precede Send; transport does not rewrite the command.
class MotionOutput {
 public:
  virtual ~MotionOutput() = default;
  virtual void Send(Twist2d command) = 0;
};
}  // namespace navigation2d::exploration
