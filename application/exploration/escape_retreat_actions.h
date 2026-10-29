#pragma once
#include <string>

namespace navigation2d::exploration {
// Immediate recovery effects. Entry state is read live after trying escape;
// callbacks may synchronously update the recovery session or entry lock.
class EscapeRetreatActions {
 public:
  virtual ~EscapeRetreatActions() = default;
  virtual bool TryStartEscape() = 0;
  virtual bool StartRetreat() = 0;
  virtual void PublishStatus(const std::string& phase, const std::string& message) = 0;
};
}  // namespace navigation2d::exploration
