#pragma once
#include <cstdint>

#include "navigation2d/application/exploration/mission_types.h"
#include "navigation2d/application/exploration/recovery_policy.h"
#include "navigation2d/application/exploration/recovery_session.h"

namespace navigation2d::exploration {
enum class EscapeContinuation { kReturn, kEntry, kResumeGoal, kDemoteGoal, kComplete, kSelect };
struct EscapeExitContext {
  MissionPhase phase;
  bool returning;
  bool entry_active;
};
// Single writer for a body-frame escape's command, deadline and return intent.
// Nanosecond stamps must use the same mission clock; zero means not scheduled.
// The caller keeps the legacy ROS duration conversion at the adapter boundary.
class EscapeExecution {
 public:
  enum class Phase { kInactive, kDriving, kExpired };
  bool Busy(std::int64_t now_ns) const { return now_ns < deadline_ns_; }
  bool scheduled() const { return deadline_ns_ != 0; }
  Phase phase(std::int64_t now_ns) const {
    if (!scheduled()) {
      return Phase::kInactive;
    }
    return Busy(now_ns) ? Phase::kDriving : Phase::kExpired;
  }
  void Begin(Twist2d command, std::int64_t deadline_ns) {
    command_ = command;
    deadline_ns_ = deadline_ns;
  }
  Twist2d Command(const BodyRanges& body) const { return AdjustEscape(command_, body); }
  // Call only after Unschedule and synchronous publication of the stop.
  // Return intent outranks a peel continuation and does not consume its budget.
  // Finishing a peel consumes its resume flag once, including demotion paths.
  EscapeContinuation AfterStop(GoalRecoverySession& recovery, EscapeExitContext context) {
    if (ConsumeReturnIntent()) {
      return EscapeContinuation::kReturn;
    }
    if (recovery.resume_after_escape) {
      recovery.resume_after_escape = false;
      if (context.phase == MissionPhase::kExploring && !context.returning) {
        return context.entry_active ? EscapeContinuation::kEntry : EscapeContinuation::kResumeGoal;
      }
      return EscapeContinuation::kDemoteGoal;
    }
    return context.phase == MissionPhase::kCompleting ? EscapeContinuation::kComplete
                                                      : EscapeContinuation::kSelect;
  }
  void ResumeReturn() { resume_return_ = true; }
  void ClearReturnIntent() { resume_return_ = false; }
  bool resume_return() const { return resume_return_; }
  // Finish only after publishing the stop, so the original side-effect order
  // (unschedule -> stop -> consume return intent) remains visible to callers.
  void Unschedule() { deadline_ns_ = 0; }
  bool ConsumeReturnIntent() {
    const bool resume = resume_return_;
    resume_return_ = false;
    return resume;
  }

 private:
  Twist2d command_{};
  std::int64_t deadline_ns_ = 0;
  bool resume_return_ = false;
};
}  // namespace navigation2d::exploration
