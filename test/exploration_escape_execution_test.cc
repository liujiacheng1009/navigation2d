#include <stdexcept>

#include "navigation2d/application/exploration/escape_execution.h"
namespace {
void Require(bool value) {
  if (!value) {
    throw std::runtime_error("escape lifecycle contract failed");
  }
}
}  // namespace
int main() {
  using namespace navigation2d::exploration;
  EscapeExecution escape;
  Require(escape.phase(100) == EscapeExecution::Phase::kInactive);
  escape.Begin({-.1, .2}, 200);
  Require(escape.scheduled() && escape.Busy(199));
  Require(escape.phase(199) == EscapeExecution::Phase::kDriving);
  Require(escape.phase(200) == EscapeExecution::Phase::kExpired);
  Require(!escape.Busy(200));
  const BodyRanges open{2., 2., 2., 2., 2.};
  const auto expected = AdjustEscape({-.1, .2}, open);
  const auto command = escape.Command(open);
  Require(command.linear == expected.linear && command.angular == expected.angular);
  escape.ResumeReturn();
  escape.Unschedule();
  Require(!escape.scheduled() && escape.resume_return());
  Require(escape.ConsumeReturnIntent());
  Require(!escape.ConsumeReturnIntent());
  escape.ResumeReturn();
  escape.ClearReturnIntent();
  Require(!escape.resume_return());
  escape.Begin({.1, -.2}, 400);
  Require(escape.phase(300) == EscapeExecution::Phase::kDriving);
  escape.Unschedule();
  GoalRecoverySession recovery;
  recovery.Begin(1., 2.);
  recovery.side_peels = 2;
  recovery.open_replans = 1;
  recovery.resume_after_escape = true;
  escape.ResumeReturn();
  Require(escape.AfterStop(recovery, {MissionPhase::kExploring, false, true}) ==
          EscapeContinuation::kReturn);
  Require(recovery.resume_after_escape && !escape.resume_return());
  Require(escape.AfterStop(recovery, {MissionPhase::kExploring, false, true}) ==
          EscapeContinuation::kEntry);
  Require(!recovery.resume_after_escape && recovery.side_peels == 2 && recovery.open_replans == 1);
  recovery.resume_after_escape = true;
  Require(escape.AfterStop(recovery, {MissionPhase::kExploring, false, false}) ==
          EscapeContinuation::kResumeGoal);
  Require(!recovery.resume_after_escape);
  for (const auto context : {EscapeExitContext{MissionPhase::kReturning, false, true},
                             EscapeExitContext{MissionPhase::kCompleting, false, false},
                             EscapeExitContext{MissionPhase::kExploring, true, false}}) {
    recovery.resume_after_escape = true;
    Require(escape.AfterStop(recovery, context) == EscapeContinuation::kDemoteGoal);
    Require(!recovery.resume_after_escape && recovery.side_peels == 2);
  }
  Require(escape.AfterStop(recovery, {MissionPhase::kCompleting, false, true}) ==
          EscapeContinuation::kComplete);
  Require(escape.AfterStop(recovery, {MissionPhase::kExploring, false, true}) ==
          EscapeContinuation::kSelect);
  Require(escape.AfterStop(recovery, {MissionPhase::kReturning, true, false}) ==
          EscapeContinuation::kSelect);
}
