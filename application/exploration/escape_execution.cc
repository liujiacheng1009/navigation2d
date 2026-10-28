#include "navigation2d/application/exploration/escape_execution.h"

#include "navigation2d/application/exploration/escape_runtime.h"
#include "navigation2d/application/exploration/motion_output.h"
#include "navigation2d/geometry/robot_geometry.h"
namespace navigation2d::exploration {
// Body-frame lidar only — no map pose / yaw. LD14 distorts hard while
// spinning and localization coasts on odom, so tip escape must prefer
// reverse translation over in-place rotate.
// Blind body-frame motion is only for a robot the planner cannot move:
// boxed in on both axes, or its latest plan failed because the start sits
// in an obstacle. Anywhere else a plan is the safer way out; escaping in
// open space drove the tail into walls it could not see.
bool EscapeExecution::TryStart(EscapeRuntime& runtime) {
  if (!runtime.EscapeInputsReady() || Busy(runtime.ExecutionNowNs())) {
    return false;
  }
  const auto body = runtime.EscapeBodyRanges();
  const bool boxed_in = HitBlocksSwing(body.front) && HitBlocksSwing(body.side);
  if (!boxed_in && !runtime.EscapeStartOccupied()) {
    runtime.ReportEscapeSkipped(body);
    return false;
  }
  const auto decision = SelectEscape(body, runtime.EscapeReturning());
  if (!decision) {
    return false;
  }
  const auto [velocity, hold_s] = *decision;
  Begin(velocity, runtime.EscapeDeadlineNs(hold_s));
  runtime.ReportEscapeStarted(body, velocity, hold_s);
  return true;
}
bool EscapeExecution::Tick(GoalRecoverySession& recovery, MotionOutput& motion,
                           EscapeRuntime& runtime) {
  if (!scheduled()) {
    return false;
  }
  if (Busy(runtime.ExecutionNowNs())) {
    // Always re-read body-frame lidar while driving, including rear closure.
    motion.Send(Command(runtime.EscapeBodyRanges()));
    runtime.EscapeDrivingStatus();
    return true;
  }
  Unschedule();
  motion.Send({});
  const auto continuation = AfterStop(recovery, runtime.EscapeExitState());
  switch (continuation) {
    case EscapeContinuation::kReturn:
      runtime.BeginReturn(true);
      break;
    case EscapeContinuation::kEntry:
      runtime.ResumeEntryAfterEscape();
      break;
    case EscapeContinuation::kResumeGoal:
      if (runtime.BreakWallScrapeAfterPeel()) {
        break;
      }
      if (runtime.NavigateAfterEscape()) {
        runtime.EscapeGoalResumed();
        break;
      }
      [[fallthrough]];
    case EscapeContinuation::kDemoteGoal:
      runtime.DemoteActiveGoal("resume after peel failed; demoting viewpoint");
      break;
    case EscapeContinuation::kComplete:
      runtime.FinishCompleting();
      break;
    case EscapeContinuation::kSelect:
      runtime.SelectAfterEscape();
      break;
  }
  return true;
}
}  // namespace navigation2d::exploration
